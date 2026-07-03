# DEPENDENCIES.md — moto_sim

All dependencies for the moto_sim motorcycle dynamics simulator.
Updated after each Milestone.  Every new dep requires a justification entry.

---

## Runtime — Python Backend

| Package | Version (min) | Purpose | Why existing stack couldn't handle it |
|---------|--------------|---------|--------------------------------------|
| `python` | 3.11+ | Runtime | f-strings, dataclasses, NamedTuple |
| `torch` (PyTorch) | 2.0+ | Pacejka MF evaluation, vectorised tire sweeps | Autograd for future PINN training; used in pinn_model.py |
| `numpy` | 1.24+ | Array math in validation tests and post-processing | Standard scientific array library |
| `scipy` | 1.10+ | Eigenvalue analysis (Milestone 2), spline fitting | Needed for `eig()`, `CubicSpline` |
| `pyyaml` | 6.0+ | Loading tire compound and bike parameter YAML files | Config-as-code; no hardcoded constants |
| `pychrono` | 8.0+ | Multibody dynamics solver (Milestone 1+) | Full DAE solver for constrained multibody; replacing hand-written chassis ODEs would require implementing Baumgarte stabilisation, constraint Jacobians, sparse linear algebra — months of work |
| `websockets` | 14.0+ | Python ↔ Electron WebSocket transport | Already in use; asyncio-native |
| `matplotlib` | 3.7+ | Post-session plots (rig validation, not used at runtime) | Standard 2D plotting |
| `pytest` | 7.0+ | Test runner | Standard |

### Install command (backend)
```bash
pip install torch numpy scipy pyyaml websockets matplotlib pytest
# PyChrono: see docs/INSTALL.md (conda or build from source)
```

---

## Runtime — Electron / Dashboard Frontend

| Package | Version | Purpose | Justification |
|---------|---------|---------|---------------|
| `electron` | 29+ | Desktop shell | Already in use; provides native window + file access |
| `react` | 18+ | UI component framework | Already in use |
| `vite` | 5+ | Frontend bundler | Already in use |
| `three` (Three.js) | 167+ | 3D wheel/motorcycle render | Already in use; WebGL scene |
| `chart.js` | 4+ | 2D plots (Fy, Mz, κ vs time) | Already in use |

### Milestone 1 additions (dashboard)
| Package | Purpose |
|---------|---------|
| `three/examples/jsm/controls/OrbitControls` | Already bundled with three.js |
| `@react-three/fiber` (optional) | React bindings for Three.js if dashboard complexity grows |

### Install command (frontend)
```bash
cd dashboard && npm install
```

---

## Development Only

| Package | Purpose |
|---------|---------|
| `pytest-cov` | Coverage reports |
| `black` | Code formatting |
| `ruff` | Linting |

---

## Milestone Additions (planned)

| Milestone | Package | Purpose |
|-----------|---------|---------|
| M4 | `scipy.interpolate.CubicSpline` | Parametric track centreline (already in scipy) |
| M5 | None planned | Thermal/wear ODEs use only numpy |
| M6 | `ffmpeg` (CLI) | Demo video capture from 3D viewport |

---

## Decisions

- **PyTorch over plain NumPy for tire model**: enables future PINN training without refactor; vectorised `forward()` enables batch tire characterisation sweeps in milliseconds.
- **PyYAML over TOML/JSON**: YAML supports comments, which are essential for physics parameter files (units, references, physical meaning).
- **websockets over ZMQ**: websockets is already installed and works natively with browser WebSocket API; ZMQ would require a separate proxy.
