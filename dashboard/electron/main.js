const { app, BrowserWindow, ipcMain } = require('electron')
const path = require('path')
const fs   = require('fs')
const { spawn } = require('child_process')

app.commandLine.appendSwitch('use-angle', 'swiftshader')
app.commandLine.appendSwitch('enable-unsafe-swiftshader')
app.commandLine.appendSwitch('disable-features', 'BlockInsecurePrivateNetworkRequests,PrivateNetworkAccessSendPreflights')

const isDev = process.env.NODE_ENV === 'development'

function createWindow() {
  const win = new BrowserWindow({
    width: 1800, height: 1020, minWidth: 1400, minHeight: 800,
    backgroundColor: '#060d18',
    title: 'Motorcycle Simulator — Live Dashboard',
    titleBarStyle: 'default',
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
    },
  })
  if (isDev) win.loadURL('http://localhost:5173')
  else       win.loadFile(path.join(__dirname, '../dist/renderer/index.html'))
}

app.whenReady().then(() => {
  createWindow()
  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow()
  })
})
app.on('window-all-closed', () => { if (process.platform !== 'darwin') app.quit() })

// ── IPC: file system for session snapshots ────────────────────────────────
ipcMain.handle('ensure-dir', (_event, dirPath) => {
  try { fs.mkdirSync(dirPath, { recursive: true }); return { ok: true } }
  catch (e) { return { ok: false, error: e.message } }
})

ipcMain.handle('save-png', (_event, base64Data, filePath) => {
  try {
    const buf = Buffer.from(base64Data.replace(/^data:image\/png;base64,/, ''), 'base64')
    fs.writeFileSync(filePath, buf)
    return { ok: true }
  } catch (e) { return { ok: false, error: e.message } }
})

ipcMain.handle('save-text', (_event, text, filePath) => {
  try { fs.writeFileSync(filePath, text, 'utf8'); return { ok: true } }
  catch (e) { return { ok: false, error: e.message } }
})

ipcMain.handle('home-dir', () => app.getPath('home'))

ipcMain.handle('list-sessions', async (_event, baseDir) => {
  try {
    if (!fs.existsSync(baseDir)) return []
    return fs.readdirSync(baseDir)
      .filter(f => fs.statSync(path.join(baseDir, f)).isDirectory())
      .sort().reverse()
  } catch { return [] }
})

// ── IPC: ROS 2 / Gazebo / SLAM stack process manager ──────────────────────
// Spawns ros2_stack/moto_ros_launch.sh <component> as a managed child process
// (its own process group) so the Robotics tab can start/stop each piece and
// stream logs. One child per component id.
const LAUNCHER = path.join(__dirname, '..', '..', 'ros2_stack', 'moto_ros_launch.sh')
const rosProcs = {}   // component -> { child, log: [] }

function rosBroadcast(channel, payload) {
  for (const w of BrowserWindow.getAllWindows()) {
    if (!w.isDestroyed()) w.webContents.send(channel, payload)
  }
}

ipcMain.handle('ros:launch', (_event, component, args = []) => {
  if (rosProcs[component]) return { ok: false, error: `${component} already running` }
  if (!fs.existsSync(LAUNCHER)) return { ok: false, error: `launcher not found: ${LAUNCHER}` }
  try {
    const child = spawn('bash', [LAUNCHER, component, ...args], {
      detached: true,                      // own process group → killpg stops ros2 launch subtree
      env: { ...process.env },
      stdio: ['ignore', 'pipe', 'pipe'],
    })
    rosProcs[component] = { child, log: [] }
    const onLine = (buf) => {
      const text = buf.toString()
      const entry = rosProcs[component]
      if (entry) { entry.log.push(text); if (entry.log.length > 400) entry.log.shift() }
      rosBroadcast('ros:log', { component, text })
    }
    child.stdout.on('data', onLine)
    child.stderr.on('data', onLine)
    child.on('exit', (code, signal) => {
      delete rosProcs[component]
      rosBroadcast('ros:exit', { component, code, signal })
    })
    return { ok: true, pid: child.pid }
  } catch (e) { return { ok: false, error: e.message } }
})

ipcMain.handle('ros:stop', (_event, component) => {
  const entry = rosProcs[component]
  if (!entry) return { ok: false, error: 'not running' }
  try { process.kill(-entry.child.pid, 'SIGINT') } catch { /* group may already be gone */ }
  // hard kill after grace period
  setTimeout(() => { try { process.kill(-entry.child.pid, 'SIGKILL') } catch { /* already dead */ } }, 2500)
  return { ok: true }
})

ipcMain.handle('ros:stopAll', () => {
  for (const c of Object.keys(rosProcs)) {
    try { process.kill(-rosProcs[c].child.pid, 'SIGINT') } catch { /* ignore */ }
  }
  return { ok: true }
})

ipcMain.handle('ros:status', () => Object.keys(rosProcs))

// ── IPC: virtual-validation suite (runs the real-physics validation in conda py) ──
const REPO = path.join(__dirname, '..', '..')
const VAL_RUNS = '/tmp/moto_validation'
function condaPython() {
  for (const p of [
    path.join(process.env.HOME || '', 'miniconda3/bin/python3'),
    path.join(process.env.HOME || '', 'anaconda3/bin/python3'),
    'python3',
  ]) { if (p === 'python3' || fs.existsSync(p)) return p }
  return 'python3'
}
const VAL_CMDS = {
  suite:       ['validation/run_suite.py', '--out', VAL_RUNS],
  correlation: ['validation/correlation.py'],
  whipple:     ['validation/whipple_benchmark.py'],
  longitudinal:['validation/benchmarks.py'],
}
const valProcs = {}
ipcMain.handle('validation:run', (_event, kind) => {
  if (!VAL_CMDS[kind]) return { ok: false, error: `unknown validation '${kind}'` }
  if (valProcs[kind]) return { ok: false, error: `${kind} already running` }
  try {
    const child = spawn(condaPython(), VAL_CMDS[kind], { cwd: REPO, env: { ...process.env } })
    valProcs[kind] = child
    const send = (buf) => {
      for (const w of BrowserWindow.getAllWindows())
        if (!w.isDestroyed()) w.webContents.send('validation:log', { kind, text: buf.toString() })
    }
    child.stdout.on('data', send)
    child.stderr.on('data', send)
    child.on('exit', (code) => {
      delete valProcs[kind]
      let report = null
      if (kind === 'suite') {
        try { report = JSON.parse(fs.readFileSync(path.join(VAL_RUNS, 'report.json'), 'utf8')) } catch { /* none */ }
      }
      for (const w of BrowserWindow.getAllWindows())
        if (!w.isDestroyed()) w.webContents.send('validation:done', { kind, code, report })
    })
    return { ok: true, pid: child.pid }
  } catch (e) { return { ok: false, error: e.message } }
})

app.on('before-quit', () => {
  for (const c of Object.keys(rosProcs)) {
    try { process.kill(-rosProcs[c].child.pid, 'SIGKILL') } catch { /* ignore */ }
  }
  for (const k of Object.keys(valProcs)) {
    try { valProcs[k].kill('SIGKILL') } catch { /* ignore */ }
  }
})
