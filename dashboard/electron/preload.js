const { contextBridge, ipcRenderer } = require('electron')

contextBridge.exposeInMainWorld('electronAPI', {
  platform: process.platform,
  version:  process.versions.electron,

  // Session snapshot file system API
  homeDir:       ()           => ipcRenderer.invoke('home-dir'),
  ensureDir:     (dir)        => ipcRenderer.invoke('ensure-dir', dir),
  savePng:       (b64, fpath) => ipcRenderer.invoke('save-png', b64, fpath),
  saveText:      (txt, fpath) => ipcRenderer.invoke('save-text', txt, fpath),
  listSessions:  (base)       => ipcRenderer.invoke('list-sessions', base),
})

// ── ROS 2 / Gazebo stack control ──────────────────────────────────────────
contextBridge.exposeInMainWorld('rosAPI', {
  isElectron: true,
  launch:  (component, args) => ipcRenderer.invoke('ros:launch', component, args),
  stop:    (component)       => ipcRenderer.invoke('ros:stop', component),
  stopAll: ()                => ipcRenderer.invoke('ros:stopAll'),
  status:  ()                => ipcRenderer.invoke('ros:status'),
  onLog:   (cb) => { const h = (_e, p) => cb(p); ipcRenderer.on('ros:log', h);  return () => ipcRenderer.removeListener('ros:log', h) },
  onExit:  (cb) => { const h = (_e, p) => cb(p); ipcRenderer.on('ros:exit', h); return () => ipcRenderer.removeListener('ros:exit', h) },
})

// ── Virtual-validation suite control ───────────────────────────────────────
contextBridge.exposeInMainWorld('validationAPI', {
  isElectron: true,
  run:    (kind) => ipcRenderer.invoke('validation:run', kind),
  onLog:  (cb) => { const h = (_e, p) => cb(p); ipcRenderer.on('validation:log', h);  return () => ipcRenderer.removeListener('validation:log', h) },
  onDone: (cb) => { const h = (_e, p) => cb(p); ipcRenderer.on('validation:done', h); return () => ipcRenderer.removeListener('validation:done', h) },
})
