/**
 * ThreeViewport — Three.js motorcycle 3-D scene.
 *
 * Coordinate mapping (PyChrono → Three.js Y-up):
 *   chrono X (fwd)  → three X
 *   chrono Y (left) → three Z
 *   chrono Z (up)   → three Y
 *
 * Oval mode: arc-length scrolling — the oval track translates + rotates each
 * frame so that the road always runs forward under the bike.  The bike stays
 * at the world origin; only the track moves.
 */
import { useRef, useEffect } from 'react'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js'

const L    = 1.370
const RF   = 0.299
const RR   = 0.318
const H_CG = 0.580
const CG_X = 0.630

function buildWheel(R, tireWidth = 0.18) {
  const group  = new THREE.Group()
  const tireMat = new THREE.MeshPhongMaterial({ color: 0x1a1a1a })
  const rimMat  = new THREE.MeshPhongMaterial({ color: 0x888888, shininess: 80 })
  const hubMat  = new THREE.MeshPhongMaterial({ color: 0xaaaaaa, shininess: 100 })
  const spkMat  = new THREE.MeshPhongMaterial({ color: 0x666666 })

  group.add(new THREE.Mesh(new THREE.CylinderGeometry(R, R, tireWidth, 18), tireMat))
  group.add(new THREE.Mesh(new THREE.CylinderGeometry(R * 0.85, R * 0.85, tireWidth * 0.92, 18), rimMat))
  group.add(new THREE.Mesh(new THREE.CylinderGeometry(R * 0.14, R * 0.14, tireWidth * 1.05, 10), hubMat))

  const stripe = new THREE.Mesh(
    new THREE.CylinderGeometry(R + 0.003, R + 0.003, 0.018, 18),
    new THREE.MeshBasicMaterial({ color: 0xff2222 }),
  )
  stripe.position.y = 0.04
  group.add(stripe)

  const spokeLen = R * 0.85 - R * 0.15
  const spokeGeo = new THREE.BoxGeometry(0.012, spokeLen, 0.012)
  for (let i = 0; i < 4; i++) {
    const a   = (i / 4) * Math.PI * 2
    const spk = new THREE.Mesh(spokeGeo, spkMat)
    const dir = new THREE.Vector3(Math.cos(a), 0, Math.sin(a))
    spk.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), dir)
    spk.position.set(dir.x * (R * 0.15 + spokeLen / 2), 0, dir.z * (R * 0.15 + spokeLen / 2))
    group.add(spk)
  }
  return group
}

function buildFrame() {
  const group = new THREE.Group()
  const mat   = new THREE.MeshPhongMaterial({ color: 0xcc2222, shininess: 40 })

  const spar = new THREE.Mesh(new THREE.BoxGeometry(CG_X, 0.06, 0.06), mat)
  spar.position.set(CG_X / 2, H_CG - RR, 0)
  group.add(spar)

  const frontSpar = new THREE.Mesh(new THREE.BoxGeometry(L - CG_X, 0.05, 0.05), mat)
  frontSpar.position.set(CG_X + (L - CG_X) / 2, H_CG - RR, 0)
  group.add(frontSpar)

  const dt1 = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, H_CG - RR, 8), mat)
  dt1.position.set(CG_X, (H_CG - RR) / 2, 0)
  group.add(dt1)

  const tank = new THREE.Mesh(
    new THREE.BoxGeometry(0.55, 0.18, 0.22),
    new THREE.MeshPhongMaterial({ color: 0xcc2222, shininess: 50 }),
  )
  tank.position.set(CG_X - 0.05, H_CG - RR + 0.12, 0)
  group.add(tank)

  return group
}

// ── Flat road ─────────────────────────────────────────────────────────────────

function addTrack(scene) {
  const trackGroup = new THREE.Group()
  scene.add(trackGroup)

  const terrain = new THREE.Mesh(
    new THREE.PlaneGeometry(140, 90),
    new THREE.MeshLambertMaterial({ color: 0xb8c79d }),
  )
  terrain.rotation.x = -Math.PI / 2
  terrain.position.y = -0.004
  trackGroup.add(terrain)

  const road = new THREE.Mesh(
    new THREE.PlaneGeometry(120, 7.2),
    new THREE.MeshLambertMaterial({ color: 0x3f4349 }),
  )
  road.rotation.x = -Math.PI / 2
  road.position.y = 0.001
  trackGroup.add(road)

  const shoulderGeom = new THREE.PlaneGeometry(120, 1.2)
  const shoulderMat  = new THREE.MeshLambertMaterial({ color: 0x657056 })
  const shoulderL    = new THREE.Mesh(shoulderGeom, shoulderMat)
  shoulderL.rotation.x = -Math.PI / 2
  shoulderL.position.set(0, 0.0005, 4.2)
  trackGroup.add(shoulderL)
  const shoulderR = shoulderL.clone()
  shoulderR.position.z = -4.2
  trackGroup.add(shoulderR)

  const stripeGroup = new THREE.Group()
  const dashGeom = new THREE.BoxGeometry(3.2, 0.02, 0.12)
  const dashMat  = new THREE.MeshBasicMaterial({ color: 0xf8fafc })
  for (let i = 0; i < 11; i++) {
    const dash = new THREE.Mesh(dashGeom, dashMat)
    dash.position.set(-30 + i * 6, 0.012, 0)
    stripeGroup.add(dash)
  }
  trackGroup.add(stripeGroup)

  const curbGeom = new THREE.BoxGeometry(4, 0.04, 0.26)
  const curbMatA = new THREE.MeshLambertMaterial({ color: 0xf5f5f5 })
  const curbMatB = new THREE.MeshLambertMaterial({ color: 0xdc2626 })
  for (let i = 0; i < 14; i++) {
    const x = -28 + i * 4.2
    const c = new THREE.Mesh(curbGeom, i % 2 === 0 ? curbMatA : curbMatB)
    c.position.set(x, 0.022, 3.62)
    trackGroup.add(c)
    const cm = c.clone()
    cm.position.z = -3.62
    trackGroup.add(cm)
  }

  return { trackGroup, stripeGroup }
}

// ── Oval track ────────────────────────────────────────────────────────────────
//
// Oval centreline (Three.js XZ, Y-up):
//   Straight1 : x  0 → Ls,  z = 0       tangent (+1, 0)
//   Corner1   : centre (Ls, Rc),          angle -π/2 → +π/2
//   Straight2 : x Ls → 0,   z = 2Rc     tangent (-1, 0)
//   Corner2   : centre (0, Rc),           angle +π/2 → +3π/2

function buildOvalPoints(Ls, Rc) {
  const pts = []
  const NS = 16, NC = 28

  for (let i = 0; i <= NS; i++)
    pts.push({ x: (i / NS) * Ls, z: 0, tx: 1, tz: 0, corner: false })

  for (let i = 1; i <= NC; i++) {
    const a = -Math.PI / 2 + (i / NC) * Math.PI
    pts.push({ x: Ls + Rc * Math.cos(a), z: Rc + Rc * Math.sin(a),
               tx: -Math.sin(a), tz: Math.cos(a), corner: true })
  }

  for (let i = 1; i <= NS; i++)
    pts.push({ x: Ls * (1 - i / NS), z: 2 * Rc, tx: -1, tz: 0, corner: false })

  for (let i = 1; i <= NC; i++) {
    const a = Math.PI / 2 + (i / NC) * Math.PI
    pts.push({ x: Rc * Math.cos(a), z: Rc + Rc * Math.sin(a),
               tx: -Math.sin(a), tz: Math.cos(a), corner: true })
  }

  return pts
}

// Precompute cumulative arc lengths; also returns total perimeter.
function buildOvalLengths(pts) {
  const cum = [0]
  for (let i = 1; i < pts.length; i++) {
    const dx = pts[i].x - pts[i - 1].x
    const dz = pts[i].z - pts[i - 1].z
    cum.push(cum[i - 1] + Math.sqrt(dx * dx + dz * dz))
  }
  const dx = pts[0].x - pts[pts.length - 1].x
  const dz = pts[0].z - pts[pts.length - 1].z
  const perim = cum[cum.length - 1] + Math.sqrt(dx * dx + dz * dz)
  return { cumLengths: cum, perim }
}

// Interpolate position + unit tangent at arc length s.
function sampleOvalAtArc(pts, cumLengths, perim, s) {
  s = ((s % perim) + perim) % perim
  let lo = 0, hi = pts.length - 1
  while (lo < hi - 1) {
    const mid = (lo + hi) >> 1
    if (cumLengths[mid] <= s) lo = mid; else hi = mid
  }
  const nxt    = (lo + 1) % pts.length
  const segLen = (cumLengths[nxt] ?? perim) - cumLengths[lo]
  const t      = segLen > 1e-9 ? (s - cumLengths[lo]) / segLen : 0
  const a = pts[lo], b = pts[nxt]
  return {
    x:  a.x  + t * (b.x  - a.x),
    z:  a.z  + t * (b.z  - a.z),
    tx: a.tx + t * (b.tx - a.tx),
    tz: a.tz + t * (b.tz - a.tz),
  }
}

// Build the oval visual and return track group + animation data.
// No grass — cleaner view to verify the oval shape.
function addOvalTrack(scene, Ls, Rc, bankAngle) {
  const trackGroup = new THREE.Group()
  scene.add(trackGroup)

  const pts  = buildOvalPoints(Ls, Rc)
  const n    = pts.length
  const hw   = 4.0
  const sinB = Math.sin(bankAngle)

  // ── Asphalt ribbon (closed quad strip, banking on corners) ────────────────
  // Left edge = inside corner = lower (y = −dY).
  // Right edge = outside corner = higher (y = +dY).
  // R_y(θ): (x,0,z) → (x cosθ + z sinθ,  0,  −x sinθ + z cosθ)
  const roadPos = []
  pts.forEach(pt => {
    const lx = -pt.tz * hw, lz = pt.tx * hw
    const dY = pt.corner ? hw * sinB : 0
    roadPos.push(pt.x + lx, -dY + 0.001, pt.z + lz)
    roadPos.push(pt.x - lx,  dY + 0.001, pt.z - lz)
  })
  const roadIdx = []
  for (let i = 0; i < n - 1; i++) {
    const [a, b, c, d] = [i * 2, i * 2 + 1, i * 2 + 2, i * 2 + 3]
    roadIdx.push(a, b, c,  b, d, c)
  }
  roadIdx.push((n - 1) * 2, (n - 1) * 2 + 1, 0,  (n - 1) * 2 + 1, 1, 0)
  const roadGeo = new THREE.BufferGeometry()
  roadGeo.setAttribute('position', new THREE.Float32BufferAttribute(roadPos, 3))
  roadGeo.setIndex(roadIdx)
  roadGeo.computeVertexNormals()
  trackGroup.add(new THREE.Mesh(roadGeo, new THREE.MeshLambertMaterial({ color: 0x3f4349 })))

  // ── Centre-line dashes every ~10 m ───────────────────────────────────────
  const dashMat = new THREE.MeshBasicMaterial({ color: 0xf8fafc })
  let dashAcc = 0
  for (let i = 0; i < n - 1; i++) {
    const dx = pts[i + 1].x - pts[i].x, dz = pts[i + 1].z - pts[i].z
    dashAcc += Math.sqrt(dx * dx + dz * dz)
    if (dashAcc >= 10) {
      dashAcc -= 10
      const pt   = pts[i]
      const dash = new THREE.Mesh(new THREE.BoxGeometry(2.5, 0.02, 0.12), dashMat)
      dash.position.set(pt.x, 0.012, pt.z)
      dash.rotation.y = Math.atan2(pt.tz, pt.tx)   // align with tangent
      trackGroup.add(dash)
    }
  }

  // ── Outer kerbs (outside/right edge), alternating red / white every 3 m ──
  const kMatA = new THREE.MeshLambertMaterial({ color: 0xf5f5f5 })
  const kMatB = new THREE.MeshLambertMaterial({ color: 0xdc2626 })
  let kAcc = 0, ki = 0
  for (let i = 0; i < n - 1; i++) {
    const dx = pts[i + 1].x - pts[i].x, dz = pts[i + 1].z - pts[i].z
    kAcc += Math.sqrt(dx * dx + dz * dz)
    if (kAcc >= 3.0) {
      kAcc -= 3.0
      const pt  = pts[i]
      const lx  = -pt.tz, lz = pt.tx         // unit left (inside)
      const ox  = pt.x - lx * hw             // outside edge centre
      const oz  = pt.z - lz * hw
      const dY  = pt.corner ? hw * sinB : 0
      const kerb = new THREE.Mesh(new THREE.BoxGeometry(3.0, 0.04, 0.28),
                                  ki % 2 === 0 ? kMatA : kMatB)
      kerb.position.set(ox, dY + 0.022, oz)
      kerb.rotation.y = Math.atan2(pt.tz, pt.tx)
      trackGroup.add(kerb)
      ki++
    }
  }

  const { cumLengths, perim } = buildOvalLengths(pts)
  return { trackGroup, stripeGroup: null, ovalPts: pts, ovalCumLengths: cumLengths, ovalPerim: perim }
}

// ── Component ─────────────────────────────────────────────────────────────────

export default function ThreeViewport({ latestRef, vehicleSetup }) {
  const mountRef    = useRef(null)
  const wheelbase   = vehicleSetup?.wheelbase ?? L
  const frontRadius = (vehicleSetup?.frontWheelDiameter ?? RF * 2) / 2
  const rearRadius  = (vehicleSetup?.rearWheelDiameter  ?? RR * 2) / 2
  const frontWidth  = vehicleSetup?.frontTyreWidth ?? 0.12
  const rearWidth   = vehicleSetup?.rearTyreWidth  ?? 0.18
  const cgHeight    = vehicleSetup?.centreOfGravityHeight ?? H_CG
  const trackMode   = vehicleSetup?.trackMode ?? 'flat'
  const straightLen = vehicleSetup?.straightLength  ?? 50
  const cornerRad   = vehicleSetup?.cornerRadius    ?? 80
  const bankAngle   = vehicleSetup?.cornerBankAngle ?? 0.15

  useEffect(() => {
    const container = mountRef.current
    if (!container) return

    const W = container.clientWidth  || 800
    const H = container.clientHeight || 500

    let renderer
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, failIfMajorPerformanceCaveat: false })
    } catch (e) {
      console.warn('[ThreeViewport] WebGL unavailable:', e.message)
      const fb = document.createElement('div')
      fb.style.cssText = 'width:100%;height:100%;display:flex;align-items:center;justify-content:center;background:#f1f5f9;color:#64748b;font:14px system-ui'
      fb.textContent = '3D view unavailable — WebGL context failed (check GPU drivers)'
      container.appendChild(fb)
      return () => { if (container.contains(fb)) container.removeChild(fb) }
    }
    renderer.setSize(W, H)
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 0.85))
    renderer.setClearColor(0xd6e0ec)
    renderer.shadowMap.enabled = false
    container.appendChild(renderer.domElement)

    const scene = new THREE.Scene()
    // Lighter fog for oval — more of the track should be visible
    scene.fog = new THREE.FogExp2(0xd6e0ec, trackMode === 'curved' ? 0.014 : 0.032)

    const camera = new THREE.PerspectiveCamera(45, W / H, 0.05, 300)
    camera.position.set(2.8, 1.45, 3.7)
    camera.lookAt(CG_X + 0.2, 0.35, 0)

    scene.add(new THREE.AmbientLight(0xffffff, 0.5))
    scene.add(new THREE.HemisphereLight(0xbcd4ff, 0x6b7355, 0.55))
    const sun = new THREE.DirectionalLight(0xfff5dd, 0.85)
    sun.position.set(4, 8, 5)
    scene.add(sun)
    const fill = new THREE.DirectionalLight(0x87b8ff, 0.22)
    fill.position.set(-4, 3, -4)
    scene.add(fill)

    // ── Gradient sky background ────────────────────────────────────────────────
    {
      const c = document.createElement('canvas')
      c.width = 2; c.height = 256
      const g = c.getContext('2d')
      const grad = g.createLinearGradient(0, 0, 0, 256)
      grad.addColorStop(0.0, '#6f9ad6')
      grad.addColorStop(0.55, '#afc6e6')
      grad.addColorStop(1.0, '#d6e0ec')
      g.fillStyle = grad; g.fillRect(0, 0, 2, 256)
      const tex = new THREE.CanvasTexture(c)
      scene.background = tex
    }

    // ── Camera modes: orbit (default) / chase / onboard (= future camera sensor POV)
    const CAM_MODES = ['orbit', 'chase', 'onboard']
    let camMode = CAM_MODES.includes(vehicleSetup?.cameraMode) ? vehicleSetup.cameraMode : 'orbit'
    const hint = document.createElement('div')
    hint.style.cssText = 'position:absolute;left:8px;bottom:8px;padding:3px 8px;border-radius:4px;' +
      'background:rgba(15,23,42,.66);color:#cbd5e1;font:11px ui-monospace,monospace;pointer-events:none;z-index:5'
    const setHint = () => { hint.textContent = `cam: ${camMode}  (press C)` }
    setHint()
    container.style.position = container.style.position || 'relative'
    container.appendChild(hint)
    const onKey = (e) => {
      if (e.key === 'c' || e.key === 'C') {
        camMode = CAM_MODES[(CAM_MODES.indexOf(camMode) + 1) % CAM_MODES.length]
        if (camMode !== 'orbit') camera.up.set(0, 1, 0)
        setHint()
      }
    }
    window.addEventListener('keydown', onKey)

    // ── Track ─────────────────────────────────────────────────────────────────
    const isOval = trackMode === 'curved'
    const trackResult = isOval
      ? addOvalTrack(scene, straightLen, cornerRad, bankAngle)
      : { ...addTrack(scene), ovalPts: null, ovalCumLengths: null, ovalPerim: 0 }

    const { trackGroup, stripeGroup, ovalPts, ovalCumLengths, ovalPerim } = trackResult

    // ── Motorcycle ────────────────────────────────────────────────────────────
    const bikeGroup  = new THREE.Group()
    scene.add(bikeGroup)

    const rearWheel  = buildWheel(rearRadius, rearWidth)
    rearWheel.rotation.x = Math.PI / 2
    bikeGroup.add(rearWheel)

    const frontWheel = buildWheel(frontRadius, frontWidth)
    frontWheel.rotation.x = Math.PI / 2
    frontWheel.position.set(wheelbase, frontRadius - rearRadius, 0)
    bikeGroup.add(frontWheel)

    bikeGroup.add(buildFrame())

    const rollBar = new THREE.Mesh(
      new THREE.CylinderGeometry(0.015, 0.015, 0.80, 8),
      new THREE.MeshBasicMaterial({ color: 0xff4444 }),
    )
    rollBar.position.set(CG_X, cgHeight - rearRadius, 0)
    bikeGroup.add(rollBar)

    // ── Orbit controls ────────────────────────────────────────────────────────
    const controls = new OrbitControls(camera, renderer.domElement)
    controls.target.set(CG_X + 0.18, 0.16, 0)
    controls.enableDamping  = false
    controls.enablePan      = false
    controls.minDistance    = 2.2
    controls.maxDistance    = 6.5
    controls.maxPolarAngle  = Math.PI * 0.48
    controls.minPolarAngle  = Math.PI * 0.22
    controls.update()

    const ro = new ResizeObserver(() => {
      const w = container.clientWidth, h = container.clientHeight
      if (w > 0 && h > 0) {
        renderer.setSize(w, h)
        camera.aspect = w / h
        camera.updateProjectionMatrix()
      }
    })
    ro.observe(container)

    // ── Animation loop ────────────────────────────────────────────────────────
    let rafId, prevMs = 0, lastRenderMs = 0
    let rSpinR = 0, rSpinF = 0
    let roadOffset = 0    // flat-road treadmill offset
    let ovalArc   = 0     // accumulated distance along oval [m]
    let prevTeleT = -1    // detect simulation reset

    const qBase  = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2)
    const qSpinR = new THREE.Quaternion()
    const qSpinF = new THREE.Quaternion()
    const spinAxis = new THREE.Vector3(0, 0, 1)

    // camera-rig scratch vectors (chase / onboard modes)
    const camPos = new THREE.Vector3(), camTgt = new THREE.Vector3(), camUp = new THREE.Vector3()
    const headLocal = new THREE.Vector3(CG_X - 0.1, (cgHeight - rearRadius) + 0.28, 0)
    const fwdLocal  = new THREE.Vector3(CG_X + 7, (cgHeight - rearRadius) + 0.05, 0)
    const chaseLocal = new THREE.Vector3(-3.4, 1.5, 0)
    const chaseTgt   = new THREE.Vector3(5, 0.35, 0)

    bikeGroup.rotation.order = 'YXZ'

    function animate(ms) {
      rafId = requestAnimationFrame(animate)
      const dt = Math.min((ms - prevMs) / 1000, 0.05)
      prevMs = ms
      if (ms - lastRenderMs < 33) return
      lastRenderMs = ms

      const f = latestRef.current
      if (f) {
        const roll  = f.roll  ?? 0
        const yaw   = f.yaw   ?? 0
        const pitch = f.pitch ?? 0
        const vx    = f.vx    ?? 0
        const teleT = f.t     ?? 0

        bikeGroup.position.set(0, rearRadius, 0)
        bikeGroup.rotation.x = roll
        bikeGroup.rotation.z = Math.max(-0.18, Math.min(0.18, pitch))

        // Wheel spin
        rSpinR += (vx / Math.max(rearRadius,  0.05)) * dt
        rSpinF += (vx / Math.max(frontRadius, 0.05)) * dt
        qSpinR.setFromAxisAngle(spinAxis, rSpinR)
        qSpinF.setFromAxisAngle(spinAxis, rSpinF)
        rearWheel.quaternion.copy(qSpinR).multiply(qBase)
        frontWheel.quaternion.copy(qSpinF).multiply(qBase)

        if (isOval) {
          // ── Oval: arc-length scrolling ──────────────────────────────────────
          // Detect simulation reset (time jumps back to 0)
          if (teleT < prevTeleT - 0.5) ovalArc = 0
          prevTeleT = teleT
          ovalArc += vx * dt

          // Find centreline point at current arc position
          const pt  = sampleOvalAtArc(ovalPts, ovalCumLengths, ovalPerim, ovalArc)

          // Rotate track so its tangent at pt aligns with world +X,
          // then translate so pt is at world origin.
          // R_y(θ): (x,0,z) → (x cosθ + z sinθ,  0,  −x sinθ + z cosθ)
          // We need θ such that (tx,tz) → (1,0):  θ = atan2(tz, tx)
          const theta = Math.atan2(pt.tz, pt.tx)
          const cosT  = Math.cos(theta), sinT = Math.sin(theta)
          trackGroup.rotation.y  = theta
          trackGroup.position.x  = -(pt.x * cosT + pt.z * sinT)
          trackGroup.position.z  =   pt.x * sinT - pt.z * cosT

          // Bike always faces world +X (track rotation handles orientation)
          bikeGroup.rotation.y = 0

        } else {
          // ── Flat: treadmill illusion ────────────────────────────────────────
          bikeGroup.rotation.y = -(yaw * 0.12)
          roadOffset = (roadOffset + vx * dt * 2.2) % 6
          if (stripeGroup) stripeGroup.position.x = -roadOffset
          trackGroup.rotation.y = -yaw * 0.03
        }

        // ── Camera rig ────────────────────────────────────────────────────────
        controls.enabled = camMode === 'orbit'
        if (camMode === 'orbit') {
          controls.update()
        } else {
          bikeGroup.updateMatrixWorld(true)
          if (camMode === 'onboard') {
            // rider POV: head position, looking forward, rolling with the bike
            camera.position.copy(bikeGroup.localToWorld(camPos.copy(headLocal)))
            camera.up.copy(camUp.set(0, 1, 0)).applyQuaternion(bikeGroup.quaternion)
            camera.lookAt(bikeGroup.localToWorld(camTgt.copy(fwdLocal)))
          } else {
            // chase: behind + above, world-up (stable to watch)
            camera.position.copy(bikeGroup.localToWorld(camPos.copy(chaseLocal)))
            camera.up.set(0, 1, 0)
            camera.lookAt(bikeGroup.localToWorld(camTgt.copy(chaseTgt)))
          }
        }
      }

      renderer.render(scene, camera)
    }

    rafId = requestAnimationFrame(animate)

    return () => {
      cancelAnimationFrame(rafId)
      ro.disconnect()
      window.removeEventListener('keydown', onKey)
      if (container.contains(hint)) container.removeChild(hint)
      controls.dispose()
      renderer.dispose()
      if (container.contains(renderer.domElement)) container.removeChild(renderer.domElement)
    }
  }, [vehicleSetup]) // eslint-disable-line react-hooks/exhaustive-deps

  return <div ref={mountRef} style={{ width: '100%', height: '100%', overflow: 'hidden' }} />
}
