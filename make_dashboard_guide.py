"""
Generate: Motorcycle_Dashboard_User_Guide.docx
Covers every panel, graph, physics formula, and R&D use case.
"""

from docx import Document
from docx.shared import Pt, RGBColor, Inches, Cm
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

doc = Document()

# ── Page setup ────────────────────────────────────────────────────────────────
for s in doc.sections:
    s.top_margin = s.bottom_margin = Cm(2.0)
    s.left_margin = s.right_margin = Cm(2.5)

# ── Style helpers ─────────────────────────────────────────────────────────────
def h1(t):
    p = doc.add_heading(t, 1)
    p.runs[0].font.color.rgb = RGBColor(31,111,235)
    p.runs[0].font.size = Pt(18)
    return p

def h2(t):
    p = doc.add_heading(t, 2)
    p.runs[0].font.color.rgb = RGBColor(13,110,253)
    p.runs[0].font.size = Pt(13)
    return p

def h3(t):
    p = doc.add_heading(t, 3)
    p.runs[0].font.color.rgb = RGBColor(40,40,40)
    p.runs[0].font.size = Pt(11)
    return p

def para(txt):
    p = doc.add_paragraph()
    p.add_run(txt).font.size = Pt(10.5)
    p.paragraph_format.space_after = Pt(4)
    return p

def bullet(txt, bold=''):
    p = doc.add_paragraph(style='List Bullet')
    if bold:
        r = p.add_run(bold)
        r.bold = True; r.font.size = Pt(10.5)
    r2 = p.add_run(txt)
    r2.font.size = Pt(10.5)
    return p

def code(txt):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Inches(0.3)
    r = p.add_run(txt)
    r.font.name = 'Courier New'; r.font.size = Pt(9)
    r.font.color.rgb = RGBColor(30,30,30)
    return p

def note(txt):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Inches(0.3)
    r = p.add_run('NOTE: '); r.bold = True; r.font.color.rgb = RGBColor(13,110,253); r.font.size=Pt(10)
    r2 = p.add_run(txt); r2.font.size=Pt(10); r2.font.color.rgb=RGBColor(50,50,50)
    return p

def warn(txt):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Inches(0.3)
    r = p.add_run('WARNING: '); r.bold=True; r.font.color.rgb=RGBColor(200,0,0); r.font.size=Pt(10)
    r2 = p.add_run(txt); r2.font.size=Pt(10); r2.font.color.rgb=RGBColor(50,50,50)
    return p

def tip(txt):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Inches(0.3)
    r = p.add_run('TIP: '); r.bold=True; r.font.color.rgb=RGBColor(0,140,50); r.font.size=Pt(10)
    r2 = p.add_run(txt); r2.font.size=Pt(10); r2.font.color.rgb=RGBColor(50,50,50)
    return p

def rd_box(txt):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Inches(0.3)
    r = p.add_run('R&D USE: '); r.bold=True; r.font.color.rgb=RGBColor(120,40,160); r.font.size=Pt(10)
    r2 = p.add_run(txt); r2.font.size=Pt(10); r2.font.color.rgb=RGBColor(50,50,50)
    return p

def lim(txt):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Inches(0.3)
    r = p.add_run('LIMITATION: '); r.bold=True; r.font.color.rgb=RGBColor(180,90,0); r.font.size=Pt(10)
    r2 = p.add_run(txt); r2.font.size=Pt(10); r2.font.color.rgb=RGBColor(50,50,50)
    return p

def table(hdr, rows, widths=None):
    t = doc.add_table(rows=1+len(rows), cols=len(hdr))
    t.style = 'Table Grid'
    t.alignment = WD_TABLE_ALIGNMENT.LEFT
    hr = t.rows[0]
    for i,h in enumerate(hdr):
        c = hr.cells[i]; c.text = h
        c.paragraphs[0].runs[0].bold = True
        c.paragraphs[0].runs[0].font.size = Pt(9)
        sh = OxmlElement('w:shd')
        sh.set(qn('w:val'),'clear'); sh.set(qn('w:color'),'auto'); sh.set(qn('w:fill'),'C8DCFF')
        c._tc.get_or_add_tcPr().append(sh)
    for ri,row in enumerate(rows):
        tr = t.rows[ri+1]
        for ci,v in enumerate(row):
            c = tr.cells[ci]; c.text = str(v)
            c.paragraphs[0].runs[0].font.size = Pt(9)
    if widths:
        for i,w in enumerate(widths):
            for row in t.rows:
                row.cells[i].width = Inches(w)
    doc.add_paragraph()
    return t

# ══════════════════════════════════════════════════════════════════════════════
# TITLE PAGE
# ══════════════════════════════════════════════════════════════════════════════
tp = doc.add_paragraph()
tp.alignment = WD_ALIGN_PARAGRAPH.CENTER
tp.paragraph_format.space_before = Pt(50)
r = tp.add_run('Motorcycle Dynamics Dashboard')
r.bold=True; r.font.size=Pt(28); r.font.color.rgb=RGBColor(31,111,235)

sp = doc.add_paragraph()
sp.alignment = WD_ALIGN_PARAGRAPH.CENTER
r2 = sp.add_run('Complete User & Engineering Guide — All Panels, All Graphs, All Physics')
r2.font.size=Pt(13); r2.font.color.rgb=RGBColor(80,80,80)

doc.add_paragraph()
vp = doc.add_paragraph()
vp.alignment = WD_ALIGN_PARAGRAPH.CENTER
r3 = vp.add_run(
    'moto_sim v1.0  ·  Python / C++ / PINN backends\n'
    'Pacejka MF-6  ·  M5 Thermal/Wear  ·  LuGre Contact Patch\n'
    'For Engineers, Researchers, and Developers'
)
r3.font.size=Pt(11); r3.font.color.rgb=RGBColor(100,100,100)

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# TABLE OF CONTENTS
# ══════════════════════════════════════════════════════════════════════════════
h1('Table of Contents')
toc = [
    ('1','System Architecture & Getting Started'),
    ('2','The Top Bar — Session Metrics & Controls'),
    ('3','Tab 1 — Live Simulation'),
    ('4','  3.1  3D Viewport'),
    ('5','  3.2  Telemetry Strip (10 channels)'),
    ('6','  3.3  Overlay Readout'),
    ('7','  3.4  Control Panel — Rider Inputs'),
    ('8','Graph Panel A — Normal Load Transfer (LoadPanel)'),
    ('9','Graph Panel B — Tyre Forces Fx / Fy (ForcePanel)'),
    ('10','Graph Panel C — Friction Circle (FrictionCircle)'),
    ('11','Graph Panel D — Slip Angles & Ratio (SlipPanel)'),
    ('12','Graph Panel E — Aligning Torque Mz (TorquePanel)'),
    ('13','Graph Panel F — Chassis Angles (ChassisPanel)'),
    ('14','Graph Panel G — Suspension Travel (SuspensionPanel)'),
    ('15','Graph Panel H — M5 Tyre Thermal & Wear (ThermalPanel)'),
    ('16','Graph Panel I — C++ 13-DOF Engine Diagnostics (Phase1Panel)'),
    ('17','Graph Panel J — Contact Patch & LuGre (Phase2Panel)'),
    ('18','Graph Panel K — PINN Coefficient Fitter (CoeffFitterPanel)'),
    ('19','Tab 2 — Parameters Workspace'),
    ('20','Tab 3 — Analysis Workspace'),
    ('21','Tab 4 — All Graphs (AllGraphsTab)'),
    ('22','Physics Deep-Dive — Pacejka Magic Formula'),
    ('23','Physics Deep-Dive — Slip, Relaxation & Camber'),
    ('24','Physics Deep-Dive — M5 Thermal, Wear & Pressure'),
    ('25','Physics Deep-Dive — LuGre Bristle Model'),
    ('26','Physics Deep-Dive — Chassis Dynamics & Stability'),
    ('27','Backend Engines — Python vs C++ vs PINN'),
    ('28','Road Conditions & Track Modes'),
    ('29','Session Recording & Data Export'),
    ('30','System Limitations'),
    ('31','R&D Use Cases & Workflow Recipes'),
    ('32','Quick Reference — Symbols, Channels & Target Ranges'),
]
for num,title in toc:
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(2)
    r1 = p.add_run(f'{num}.  '); r1.bold=True; r1.font.size=Pt(10.5)
    r2 = p.add_run(title); r2.font.size=Pt(10.5)

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# 1 — SYSTEM ARCHITECTURE
# ══════════════════════════════════════════════════════════════════════════════
h1('1. System Architecture & Getting Started')

h2('What is moto_sim?')
para(
    'moto_sim is a real-time motorcycle vehicle-dynamics simulator with a live telemetry dashboard. '
    'A physics backend (Python or C++) runs at 1 kHz and streams ~50 telemetry channels at 50 Hz over '
    'WebSocket to an Electron/React dashboard. The dashboard visualises every channel as live time-series '
    'graphs, a 3D viewport, a friction circle, and analytical workspaces — with zero post-processing delay.'
)

h2('Architecture Overview')
table(
    ['Layer','Technology','Role'],
    [
        ['Physics backend (Python)','moto_sim_main.py · Pacejka MF · M5 thermal','1 kHz simulation loop. Computes all tyre forces, chassis motion, thermal state.'],
        ['Physics backend (C++)','cpp_rk4 · RK4 integrator · LibTorch','13-DOF rigid-body model at ~700× real-time. Faster, adds dynamic trail/wheelbase.'],
        ['PINN tire model','ONNX tire_pinn_medium.onnx · OnnxTireModel','Neural-network replacement for the Pacejka Fy / Mz lookup, trained on simulation data.'],
        ['Transport','WebSocket ws://localhost:8765','Flat JSON frames at 50 Hz. Commands (throttle, steer, reset) sent from dashboard to backend.'],
        ['Dashboard','Electron + React + Recharts/Chart.js + Three.js','Renders all graphs, 3D model, controls. No physics computed client-side.'],
    ],
    widths=[1.5,2.0,3.0]
)

h2('Launching the Application')
para('Double-click the "Motorcycle Dashboard" icon on the Desktop, or run:')
code('bash motorcycle_dashboard_launcher.sh')
para('A backend selection dialog appears with five engine options:')
table(
    ['Option','Engine','Best for'],
    [
        ['Python Simulation','moto_sim_main.py · Pacejka MF + M5 thermal','Default: full thermal/wear model, most validated'],
        ['C++ 13-DOF Engine','cpp_rk4 binary via run_engine.py bridge','Speed: 700× real-time, dynamic trail/wheelbase, Pacejka'],
        ['C++ 13-DOF + PINN','C++ engine + ONNX neural tire model','Research: neural tire replaces analytical Pacejka'],
        ['C++ Phase 2 Tire','FullMF + LuGre bristle + Hertzian contact patch','Advanced: contact patch area, sliding fraction live'],
        ['C++ FullMF Tire','Large-camber Pacejka + gyroscopic Mz + turn-slip','Most complete: camber thrust, turn-slip, gyroscopic Mz'],
    ],
    widths=[1.8,2.3,2.4]
)
note('The launcher waits until the WebSocket port 8765 opens before starting the Electron window. '
     'If the backend crashes, check /tmp/motorcycle_dashboard_logs/simulator.log.')

h2('Default Bike Parameters (CBR600RR baseline)')
table(
    ['Parameter','Default value','Unit'],
    [
        ['Wheelbase','1.37','m'],
        ['Front wheel diameter','0.598','m'],
        ['Rear wheel diameter','0.636','m'],
        ['Front tyre width','0.12','m'],
        ['Rear tyre width','0.18','m'],
        ['Rake angle','24','°'],
        ['Trail','0.097','m'],
        ['CoG height','0.58','m'],
        ['Total mass','189','kg'],
        ['Front weight fraction','0.46','—'],
        ['Engine torque max','80','N·m'],
        ['Brake torque max','250','N·m'],
        ['Front spring rate','18 500','N/m'],
        ['Rear spring rate','42 000','N/m'],
        ['Front damping','900','N·s/m'],
        ['Rear damping','1 800','N·s/m'],
        ['Friction coefficient (μ)','0.9','—'],
        ['Initial speed (v₀)','12','m/s'],
    ],
    widths=[2.5,1.5,0.8]
)

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# 2 — TOP BAR
# ══════════════════════════════════════════════════════════════════════════════
h1('2. The Top Bar — Session Metrics & Controls')
para(
    'The top bar is always visible regardless of which tab is active. '
    'It contains brand identity, session counters, four hero metrics, and action buttons.'
)

h2('Session Info Chips')
table(
    ['Chip','What it shows','Colour alert'],
    [
        ['SESSION','Elapsed time mm:ss since app opened','—'],
        ['FRAMES','Total WebSocket frames received (#n)','—'],
        ['FPS','Current WebSocket frame rate','Red < 20, Amber < 40, Green ≥ 40'],
        ['REC (when active)','Red dot — session is being recorded','Red border = recording active'],
    ],
    widths=[1.2,3.0,2.2]
)

h2('Hero Metrics (always live)')
table(
    ['Metric','Channel','Unit','Alert condition'],
    [
        ['Speed','vx','m/s','—'],
        ['Lean','roll (radians → °)','°','Red if |roll| > 0.5 rad (≈ 28.6°)'],
        ['Yaw ω','yaw_rate (rad/s → °/s)','°/s','—'],
        ['Fz F/R','f_Fz / r_Fz','N','Amber if front > 65% or front < 30% of total'],
    ],
    widths=[1.2,2.0,0.8,2.4]
)
note('All angles arrive from the backend in RADIANS. The dashboard multiplies by 180/π (R2D) for display.')

h2('Engine Badge')
para('A colour-coded badge shows which backend is active:')
table(
    ['Badge colour','Engine'],
    [
        ['Grey','Python Pacejka simulation'],
        ['Cyan','C++ 13-DOF (base)'],
        ['Purple','C++ 13-DOF + PINN neural tire'],
        ['Green','Phase 2 / FullMF tire'],
    ],
    widths=[1.5,4.8]
)

h2('Action Buttons')
for item,desc in [
    ('⏺ REC / ⏹ STOP','Start or stop session recording. On stop, saves full telemetry buffer to disk as CSV + JSON.'),
    ('🏁 (Lap marker)','Increment the lap counter. Displayed next to the button. Used to mark circuit laps for comparative analysis.'),
    ('☀️ / 🌙','Toggle dark / light theme. Applies immediately to all panels.'),
]:
    bullet(desc, bold=item+': ')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# 3 — LIVE TAB
# ══════════════════════════════════════════════════════════════════════════════
h1('3. Tab 1 — Live Simulation')
para(
    'The Live tab (default) is split into two areas: a LEFT panel containing the 3D viewport, '
    'telemetry strip, and rider control panel; and a RIGHT grid of live data panels (graphs). '
    'Every graph in the right grid has an ⤢ expand button to open it full-screen.'
)

h2('3.1  3D Viewport')
para(
    'A Three.js 3D model of the motorcycle updates in real time at the WebSocket frame rate (50 Hz). '
    'The model shows chassis lean (roll), pitch, and steering angle visually. '
    'The frame-rate badge (top-right of the viewport) confirms the update speed.'
)
para('Controls:')
for item in ['Left-click + drag — orbit / rotate view','Scroll — zoom','Right-click + drag — pan']:
    bullet(item)
rd_box(
    'Use the 3D viewport to quickly visually confirm that the chassis response matches '
    'expected behaviour before diving into the graphs — e.g. does the bike lean into a corner correctly?'
)

h2('3.2  Telemetry Strip — 10 Live Channels')
para('A row of 10 numeric readouts updates at 10 Hz (React poll interval):')
table(
    ['Channel','Symbol','Unit','Alert'],
    [
        ['SPEED','vx','m/s','—'],
        ['LEAN','roll','°','Red if |roll| > 0.5 rad'],
        ['YAW ω','yaw_rate','°/s','—'],
        ['STEER δ','steer_angle','°','—'],
        ['FZ FRONT','f_Fz','N','Amber if front fraction > 65%'],
        ['FZ REAR','r_Fz','N','Amber if front fraction < 30%'],
        ['Fy FRONT','f_Fy','N','—'],
        ['ROAD μ','road_mu_f','—','Amber if < 0.7 (low grip); Green if ≥ 0.7'],
        ['TRAIL (C++ only)','effective_trail × 1000','mm','—'],
        ['TIME','t','s','—'],
    ],
    widths=[1.3,1.3,0.7,2.8]
)

h2('3.3  Overlay Readout (inside 3D viewport)')
para('A semi-transparent panel in the top-right of the 3D viewport repeats nine key values in real time:')
for ch,col in [
    ('time [s]','dim grey'),('speed [m/s]','cyan'),('lean [°]','red'),
    ('yaw rate [°/s]','blue'),('steer δ [°]','amber'),
    ('Fz front [N]','green'),('Fz rear [N]','cyan'),
    ('Fz ratio [%F]','red/amber/grey depending on balance'),
    ('road μ','violet'),
]:
    bullet(f'{ch}  — colour: {col}')

h2('3.4  Control Panel — Rider Inputs')
para(
    'The Control Panel sits below the 3D viewport. It sends commands to the backend over WebSocket. '
    'Every slider change is transmitted immediately; the physics sees it at the next 1 kHz step.'
)
table(
    ['Control','Range','Step','Units','Behaviour'],
    [
        ['Throttle','0 – 1','0.02','%','Engine torque applied = throttle × engine_torque_max. Green when > 50%.'],
        ['Brake Front','0 – 1','0.02','%','Front brake torque = brake_f × brake_torque_max × 0.7. Red when > 0%.'],
        ['Brake Rear','0 – 1','0.02','%','Rear brake torque = brake_r × brake_torque_max × 0.3. Red when > 0%.'],
        ['Steer torque','−30 to +30','0.5','N·m','Handlebar torque input. Auto-returns to 0 on release. Amber if |τ| > 5.'],
        ['Lean δ','−17 to +17','0.5','°','Rider upper-body lean command. Violet if |δ| > 8°.'],
        ['Target Speed (CC)','0 – 30','0.5','m/s','Cruise-control speed. Green border = active.'],
        ['v₀','1 – 25','0.5','m/s','Initial speed when Reset is pressed.'],
        ['Road','4 options','—','—','Sets surface friction μ: Dry=1.0, Wet=0.6, Race=1.1, Mountain=0.9.'],
    ],
    widths=[1.5,1.1,0.6,0.7,2.6]
)
note(
    'Steer torque and lean δ are rider inputs into the CONTROLLER, not direct state variables. '
    'The simulation integrates the equations of motion from these inputs.'
)
rd_box(
    'Sweep the Throttle from 0 to 1 gradually while watching the Friction Circle (Panel C) '
    'to observe how longitudinal slip (κ) consumes the available traction budget. '
    'This is traction circle utilisation analysis — critical for drive/brake allocation.'
)

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL A — LOAD
# ══════════════════════════════════════════════════════════════════════════════
h1('4. Graph Panel A — Normal Load Transfer')

h2('What This Graph Shows')
para(
    'Two time-series lines (updated every 140 ms) plot the vertical (normal) load on each tyre. '
    'A live weight-distribution bar below the chart shows the instantaneous front/rear split as a percentage.'
)

h2('Chart Specification')
table(
    ['Element','Detail'],
    [
        ['X axis','Time [s] — scrolling window of recent history'],
        ['Y axis','Normal load Fz [N] — minimum clamped to 0'],
        ['Fz_front (lime #a3e635)','Vertical load on front tyre contact patch'],
        ['Fz_rear (cyan #22d3ee dashed)','Vertical load on rear tyre contact patch'],
        ['Live bar','Front% / Rear% split — red if front > 65%, orange if front < 30%, cyan otherwise'],
    ],
    widths=[2.0,4.4]
)

h2('Physics — How Normal Load Is Computed')
para('Under acceleration or braking, the static axle loads shift forward or rearward:')
code('Static:   Fz_front_static = M × g × (L − x_cg) / L')
code('Static:   Fz_rear_static  = M × g × x_cg / L')
code('Dynamic:  ΔFz = M × a × h_cg / L         (weight transfer)')
code('Braking:  Fz_front = Fz_static_f + ΔFz   (increases)')
code('Braking:  Fz_rear  = Fz_static_r − ΔFz   (decreases)')
para('Where:')
table(
    ['Symbol','Meaning','Typical value'],
    [
        ['M','Total mass (bike + rider)','189 + 75 = 264 kg'],
        ['g','Gravitational acceleration','9.81 m/s²'],
        ['L','Wheelbase','1.37 m'],
        ['x_cg','CoG distance from front axle','L × front_fraction = 0.63 m'],
        ['h_cg','CoG height above ground','0.58 m'],
        ['a','Longitudinal acceleration (+accel, −brake)','m/s²'],
    ],
    widths=[1.0,3.0,2.4]
)

h2('How to Read the Graph')
for item in [
    'Both lines should be non-zero at all times — if Fz drops to zero, that tyre is lifting (wheelie or stoppie)',
    'During hard braking the lime line rises and the cyan line falls — front load increases',
    'During acceleration the opposite occurs — rear load rises',
    'Cornering adds minimal change to Fz total but redistributes laterally (not visible in this 2D plot)',
    'The red live bar (> 65% front) warns of excessive dive — check brake pressure and suspension settings',
]:
    bullet(item)

h2('Target Ranges')
table(
    ['Condition','Front % target','Action if out of range'],
    [
        ['Static (sitting still)','42–55%','Adjust rider position or fuel load in Parameters tab'],
        ['0.5 g braking','55–70%','Check anti-dive geometry (rake, trail)'],
        ['0.8 g braking (hard stop)','65–78%','Normal for sport geometry'],
        ['Full throttle acceleration','30–44%','If < 30%: wheelie risk — reduce throttle or lower CoG'],
    ],
    widths=[2.0,1.5,3.0]
)

h2('R&D Applications')
rd_box('Suspension spring rate optimisation: sweep the front spring rate in the Parameters tab '
       'and watch peak Fz_front under a repeatable 0.8 g brake event. Minimum spring rate = peak Fz / max deflection.')
rd_box('Anti-squat geometry study: compare Fz_rear droop during aggressive throttle application across '
       'different CoG height settings. Higher CoG → more rear load under acceleration.')
lim('The model uses a lumped-mass weight transfer formula. It does not model left-right load transfer '
    'in cornering (no lateral CoG movement). The bar chart always shows fore-aft balance only.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL B — FORCES
# ══════════════════════════════════════════════════════════════════════════════
h1('5. Graph Panel B — Tyre Forces Fx / Fy')

h2('What This Graph Shows')
para(
    'Four time-series channels (updated every 140 ms) show the longitudinal (Fx) and lateral (Fy) '
    'tyre forces at front and rear. These are the forces computed by the Pacejka Magic Formula '
    'and passed to the chassis equations of motion.'
)

h2('Chart Specification')
table(
    ['Channel','Colour','Description'],
    [
        ['Fx_front','Lime #a3e635 solid','Front tyre longitudinal force (+ = forward drive, − = braking)'],
        ['Fx_rear','Cyan #22d3ee solid','Rear tyre longitudinal force'],
        ['Fy_front','Orange #f97316 dashed','Front tyre lateral (cornering) force'],
        ['Fy_rear','Purple #c084fc dashed','Rear tyre lateral (cornering) force'],
    ],
    widths=[1.3,1.8,3.5]
)
para('X axis: Time [s]  |  Y axis: Force [N]')

h2('Physics — Pacejka Magic Formula (Brief)')
para('The Pacejka Magic Formula (MF-6) computes Fx and Fy from slip inputs:')
code('Pure lateral:  Fy_0 = D · sin(C · atan(B·α − E·(B·α − atan(B·α))))')
code('Pure long.:    Fx_0 = μ·Fz·tanh(Kx·κ)')
code('Combined:      Fy   = Gyκ · Fy_0       Fx = Gxα · Fx_0')
code('               Gyκ  = cos(Cyk · atan(Byk · κ))    ← Fy drops when κ ≠ 0')
code('               Gxα  = cos(Cxa · atan(Bxa · α))    ← Fx drops when α ≠ 0')
para('Key: D = μ × Fz is the PEAK force. The combined-slip G-factors (both ≤ 1) reduce it when both slip angle and slip ratio are non-zero simultaneously.')

h2('How to Read the Graph')
for item in [
    'Fx_rear positive (lime rises) during throttle application — check magnitude vs brake force',
    'Fy non-zero indicates cornering — front and rear Fy should have the same sign in a steady turn',
    'If Fy_front and Fy_rear have opposite signs, the bike is in a drift or oscillating — check stability',
    'Both Fx and Fy drop when Fz drops (less normal load → less peak force D = μ × Fz)',
    'Spikes in Fx indicate ABS-like events (wheel slip cycling)',
]:
    bullet(item)

h2('Target Ranges')
table(
    ['Force','Typical peak magnitude','Exceeding this means'],
    [
        ['Fx_rear (drive)','0 – 800 N','> 1000 N: approaching friction limit, risk of wheelspin'],
        ['Fx (braking)','0 – −1500 N at the wheel','More than μ×Fz_dynamic → wheel lock'],
        ['Fy (cornering)','0 – 1800 N per tyre','Beyond μ×Fz → tyre saturation, understeer/oversteer'],
    ],
    widths=[1.8,2.0,2.8]
)

rd_box('Compare Fx_rear vs Fy_rear during a corner-exit acceleration event. '
       'The ratio Fy/Fz vs Fx/Fz (friction circle) shows traction budget split. '
       'Target: Fx²+Fy² < (μFz)² at all times.')
lim('The simulation runs 5-body (front/rear wheel + chassis) with point-contact tyres. '
    'Real motorcycles have contact-patch length effects not captured in pure Pacejka.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL C — FRICTION CIRCLE
# ══════════════════════════════════════════════════════════════════════════════
h1('6. Graph Panel C — Friction Circle')

h2('What This Graph Shows')
para(
    'The Friction Circle (also called the g-g diagram or traction circle) plots normalised tyre '
    'force in two dimensions simultaneously. It is THE most powerful single graph for understanding '
    'combined tyre utilisation. Both beginners and experts should understand this graph deeply.'
)

h2('Chart Specification')
table(
    ['Element','Detail'],
    [
        ['X axis','μx = Fx / Fz  [dimensionless, range −1.2 to +1.2]'],
        ['Y axis','μy = Fy / Fz  [dimensionless, range −1.2 to +1.2]'],
        ['Operating locus (light blue)','Trail of recent (μx, μy) pairs — shows tyre utilisation history'],
        ['Current point (yellow)','Latest single data point — where we are right now'],
        ['Friction circle (pink ring)','Radius = TIRE_MU × road_μ = 0.9 × μ_road — the limit boundary'],
    ],
    widths=[2.0,4.4]
)

h2('Physics — What the Circle Means')
para(
    'Each tyre can produce a maximum resultant force equal to μ × Fz (the friction limit). '
    'When split into X and Y components:'
)
code('Resultant = √(Fx² + Fy²)   ≤   μ × Fz      (friction ellipse, idealised as circle here)')
code('μx = Fx/Fz    (normalised longitudinal coefficient)')
code('μy = Fy/Fz    (normalised lateral coefficient)')
code('Limit circle radius = TIRE_MU × road_mu_f    (= 0.9 × μ_road in the model)')

h2('Reading the Friction Circle — Step by Step')
table(
    ['Position on diagram','Physical meaning','What to do'],
    [
        ['Centre (0, 0)','No force — straight line, no accel/brake','Coasting — full traction budget available'],
        ['Top (μy > 0)','Pure right-hand cornering','Lateral load — reduce speed if near the circle'],
        ['Bottom (μy < 0)','Pure left-hand cornering',''],
        ['Right (μx > 0)','Pure drive/throttle','Rear wheel driving'],
        ['Left (μx < 0)','Pure braking','Deceleration'],
        ['Top-left quadrant','Braking while turning left (trail braking)','Most demanding — limit easy to exceed'],
        ['On the circle boundary','Tyre at friction limit','Any more force → slip, loss of control'],
        ['Outside the circle','Impossible (model clamped) / physical tyre has exceeded limit','Emergency — tyre has lost grip'],
    ],
    widths=[1.8,2.0,2.6]
)

h2('Beginners Guide to This Graph')
para('Think of the circle as a pizza. You can use the pizza any way you like, but you cannot eat more than one pizza:')
for item in [
    'Eating all the pizza vertically (full cornering) means no pizza left for braking or acceleration',
    'Eating diagonally (cornering + braking) uses the pizza from two directions at once',
    'The yellow dot shows how much pizza you are using right now',
    'If the yellow dot hits the circle edge, you have run out of pizza — the tyre will slide',
]:
    bullet(item)

h2('R&D Applications')
rd_box('Trail braking analysis: during a corner entry, watch the locus move from the top (cornering) '
       'diagonally toward bottom-left (braking + cornering). Optimal trail braking stays just inside the circle.')
rd_box('Friction coefficient measurement: on an unknown road surface, apply a calibrated brake input '
       'and read the peak |μx| achieved. This estimates the actual road μ.')
rd_box('Combined slip validation: compare the circle shape to Pacejka coefficient predictions. '
       'A flattened ellipse vs a circle reveals Cxα / Cyk coefficient errors.')
lim('The circle here represents the FRONT tyre only (f_Fz, f_Fy, f_Fx). '
    'Rear tyre has its own budget — add a second circle for rear analysis in a future enhancement.')
lim('The model uses a circular friction limit. Real tyres have a slightly elliptical limit '
    '(longitudinal peak force ≠ lateral peak force). The Phase 2 / FullMF backend models this more accurately.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL D — SLIP
# ══════════════════════════════════════════════════════════════════════════════
h1('7. Graph Panel D — Slip Angles & Ratio')

h2('What This Graph Shows')
para(
    'A dual-axis time-series chart showing tyre slip angle (α) on the left axis and '
    'longitudinal slip ratio (κ) on the right axis for both front and rear tyres. '
    'Updated every 140 ms.'
)
table(
    ['Channel','Axis','Colour','Description'],
    [
        ['α_front','Left [°]','Purple #7c3aed solid','Front tyre slip angle'],
        ['α_rear', 'Left [°]','Purple #c084fc dashed','Rear tyre slip angle'],
        ['κ_front','Right [—]','Orange #ea580c solid','Front tyre slip ratio'],
        ['κ_rear', 'Right [—]','Orange #f97316 dashed','Rear tyre slip ratio'],
    ],
    widths=[1.3,1.0,1.8,2.5]
)

h2('Physics — Slip Angle α')
code('α = −atan2(vy_tyre, vx_tyre)')
para(
    'Slip angle is the angle between the tyre heading direction and the actual velocity vector of '
    'the contact patch. It arises during cornering when the tyre must generate lateral force.'
)
table(
    ['α value','Physical state','Lateral force'],
    [
        ['0°','Tyre rolling straight — no lateral force','0 N'],
        ['1–3°','Normal cornering — well within the linear range','Rising with α'],
        ['5–8°','Near peak — Pacejka peak cornering force','≈ μ × Fz'],
        ['> 10°','Saturation / sliding — tyre overloaded','Decreasing or plateau'],
    ],
    widths=[1.2,3.2,1.8]
)

h2('Physics — Longitudinal Slip Ratio κ')
code('κ = (R_w · ω_w − vx) / max(|vx|, ε)    for drive (ω too fast)')
code('κ = (R_w · ω_w − vx) / max(R_w · ω_w, ε)   for braking (ω too slow)')
para(
    'κ = 0 means the tyre rolls freely with no slip. κ > 0 means the wheel is spinning faster than '
    'the vehicle (drive slip). κ < 0 means the wheel is slower than the vehicle (brake slip / lock).'
)
table(
    ['κ value','Physical state','Longitudinal force'],
    [
        ['0','Free rolling','0 Fx (no drive or brake force)'],
        ['0 to 0.1','Light acceleration / braking — linear range','Rising with κ'],
        ['0.1 to 0.2','Near peak longitudinal force','≈ μ × Fz'],
        ['> 0.2 drive','Wheelspin — drive slip exceeds peak','Decreasing — traction loss'],
        ['< −0.2 brake','Wheel lockup approaching','Decreasing — brake effectiveness drops'],
    ],
    widths=[1.5,2.5,2.2]
)

h2('Slip Relaxation — Why α Lags Behind')
para(
    'The tyre does not respond to slip angle instantaneously. The simulation uses a ZOH (zero-order hold) '
    'relaxation ODE:'
)
code('α′[k+1] = α_kinematic + (α′[k] − α_kinematic) · exp(−vx·dt / σ)')
para('σ = relaxation length ≈ 0.30 m for a motorcycle tyre. At 10 m/s, the time constant τ = σ/vx = 30 ms. '
     'At higher speeds, the tyre responds faster (smaller time constant). This causes the lag visible on the graph.')

h2('How to Read the Graph')
for item in [
    'α near zero: straight-line riding — no cornering load',
    'α rising from zero: entering a corner — watch α_front lead α_rear',
    'α_front > α_rear at steady state: mild understeer (typical for motorcycles)',
    'κ spikes positive: throttle blip or wheelspin event',
    'κ spikes negative: hard braking or lock-up event',
    'Large α AND large κ simultaneously: combined-slip event — both G-factors active',
]:
    bullet(item)

rd_box('Self-steering torque measurement: at constant speed cornering, plot α_front vs Mz_front (Panel E). '
       'The slope dMz/dα gives the effective pneumatic trail — a tyre identification metric.')
lim('The model uses a single-point contact with relaxation. It does not model belt/carcass '
    'belt dynamics (transient response faster than σ/vx) or the contact-patch tilt.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL E — TORQUE
# ══════════════════════════════════════════════════════════════════════════════
h1('8. Graph Panel E — Aligning Torque Mz')

h2('What This Graph Shows')
para(
    'Two time-series channels (updated every 140 ms) plot the self-aligning torque Mz at the '
    'front and rear contact patches. The front Mz channel has a fill: red above zero (Mz acts against '
    'the steering input) and green below zero (Mz supports the steering).'
)
table(
    ['Channel','Colour','Fill'],
    [
        ['Mz_front [N·m]','Green #34d399 solid','Red fill above zero, green fill below zero'],
        ['Mz_rear [N·m]', 'Purple #818cf8 dashed','No fill'],
    ],
    widths=[1.8,2.0,2.6]
)

h2('Physics — What Is Mz?')
para(
    'Mz is the torque that a tyre exerts about its vertical axis at the contact patch. '
    'It is produced by the offset between the lateral force application point (pneumatic trail) '
    'and the contact patch centre.'
)
code('tp   = tp0 · cos(Ct · atan(Bt · α′))    [pneumatic trail in m]')
code('Mz   = −Fy · tp  +  K_gamma_Mz · γ · Fz')
para('Where:')
table(
    ['Symbol','Meaning','Typical value'],
    [
        ['tp0','Maximum pneumatic trail (at α=0)','0.030 m = 30 mm'],
        ['Bt, Ct','Trail curve shape factors','Bt=10, Ct=1.5'],
        ['α′','Relaxed slip angle [rad]','Current from relaxation ODE'],
        ['γ','Camber angle [rad]','Lean angle of tyre from vertical'],
        ['K_gamma_Mz','Camber-induced Mz coefficient','0.020'],
        ['Fz','Normal load [N]','Computed per step'],
    ],
    widths=[1.3,3.0,2.0]
)

h2('How Pneumatic Trail Changes with Slip')
table(
    ['Slip angle α','tp','Mz direction','Rider feel'],
    [
        ['0°','Maximum (tp0 = 30 mm)','Mz = 0 (no Fy)','Straight ahead — no feedback'],
        ['Small (1–3°)','Near maximum','Mz < 0 (tries to straighten)','Steering "feels heavy"'],
        ['At Fy peak (≈5–8°)','Dropping toward 0','|Mz| reduces','Steering "goes light" — saturation warning'],
        ['> α_peak','tp < 0 (reversal!)','Mz > 0 (destabilises)','DANGER — tyre past saturation'],
    ],
    widths=[1.5,1.5,2.0,2.4]
)
warn('Mz reversal (Mz_front going positive while cornering) indicates the front tyre has passed its '
     'peak slip angle and is beginning to lose grip. This is the critical warning before a front-wheel slide.')

h2('How to Read the Graph')
for item in [
    'Mz_front oscillating between small negative values: normal steady cornering',
    'Mz_front approaching zero then going positive: front tyre saturation — release throttle, reduce lean angle',
    'Mz_rear large and positive: rear tyre counter-steering or near slide',
    'Both Mz channels near zero at high speed: tyre in combined slip — check friction circle simultaneously',
    'The red fill zone on Mz_front is the "safe restoring" regime — steering torque helps the rider correct',
]:
    bullet(item)

rd_box('Pneumatic trail measurement: at constant Fz (flat road), plot |Mz_front| / |Fy_front| vs α_front. '
       'The ratio gives empirical tp(α). Compare to the Pacejka tp formula to validate coefficients.')
lim('Mz_rear is shown for completeness but the rear tyre does not directly feed back to the rider '
    '(no steering link). It affects the yaw dynamics but is rarely monitored in real testing.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL F — CHASSIS
# ══════════════════════════════════════════════════════════════════════════════
h1('9. Graph Panel F — Chassis Angles')

h2('What This Graph Shows')
para(
    'Five time-series channels (updated every 140 ms) show the three chassis orientation angles '
    '(roll, pitch, yaw) and two angular rates (roll rate, yaw rate) on dual Y-axes.'
)
table(
    ['Channel','Axis','Colour','Unit'],
    [
        ['Roll angle φ','Left [°]','Red #ef4444 solid','Lean angle from vertical (positive = right lean)'],
        ['Pitch angle θ','Left [°]','Amber #f59e0b dashed [4,3]','Nose up/down (positive = nose up)'],
        ['Yaw angle ψ','Left [°]','Blue #3b82f6 dashed [2,3]','Heading change (cumulative)'],
        ['Roll rate φ̇','Right [°/s]','Light red #f87171 solid','Rate of lean change'],
        ['Yaw rate ψ̇','Right [°/s]','Light blue #60a5fa solid','Turn rate (most important for stability)'],
    ],
    widths=[1.5,1.0,2.0,2.0]
)

h2('Physics — The 3-DOF Chassis Model')
para('The chassis is modelled as a rigid body with three rotational degrees of freedom:')
code('Roll (φ):  lean of the motorcycle left/right — primary stability variable')
code('Pitch (θ): nose-up under acceleration, nose-down under braking')
code('Yaw (ψ):  heading direction change — integral of yaw_rate')
para('The equations of motion (simplified Whipple model) couple roll and yaw:')
code('I_roll · φ̈ = M·g·h·sin(φ) − M·v·ψ̇·h·cos(φ) + rider_lean_torque')
code('I_yaw  · ψ̈ = (Fy_f·a + Fy_r·b) / L')

h2('How to Read the Graph')
table(
    ['Observation','Interpretation'],
    [
        ['Roll angle steady at non-zero value','Steady-state cornering — normal'],
        ['Roll angle oscillating (wobble)','High-frequency oscillation = wobble mode (steering oscillation, typically 6–10 Hz)'],
        ['Roll angle diverging slowly','Capsize mode — static instability at low speed'],
        ['Large yaw_rate with small roll_rate','Good — turning without excessive lean change'],
        ['Roll_rate and yaw_rate in phase','Weave mode — coupled oscillation, dangerous if amplitude grows'],
        ['Pitch rising sharply then falling','Wheelie attempt — monitor Fz_front approaching zero simultaneously'],
    ],
    widths=[2.5,4.0]
)
note('The stability threshold is speed-dependent. Capsize mode is unstable at low speed; '
     'weave and wobble typically become unstable above certain speeds depending on geometry.')
rd_box('Stability mode identification: excite the bike with a steer pulse (brief steer torque input) '
       'then release. Count the oscillation frequency in the roll and yaw_rate channels. '
       'Compare to the Analysis tab predictions for wobble (6–10 Hz) and weave (2–4 Hz).')
lim('The dashboard shows roll, pitch, yaw as Euler angles. Gimbal lock occurs at ±90° pitch — '
    'irrelevant for motorcycles on road surfaces but would be an issue for extreme stunts.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL G — SUSPENSION
# ══════════════════════════════════════════════════════════════════════════════
h1('10. Graph Panel G — Suspension Travel')

h2('What This Graph Shows')
para(
    'Two time-series channels (updated every 140 ms) show the compression/extension of front fork '
    'and rear shock absorber in millimetres. Positive = compressed.'
)
table(
    ['Channel','Colour','Description'],
    [
        ['Fork travel [mm]','Amber #f59e0b solid','Front fork compression from static ride height'],
        ['Rear travel [mm]','Purple #818cf8 dashed [5,2]','Rear shock compression from static ride height'],
    ],
    widths=[1.8,1.8,3.0]
)
para('Backend sends travel in metres; dashboard multiplies × 1000 to display in mm.')

h2('Physics — Suspension Model')
code('F_spring = k × x        (spring force, N)')
code('F_damper = c × ẋ        (damper force, N)')
code('F_suspension = F_spring + F_damper')
code('Net chassis force: chassis sees road through suspension — wheel stays on ground')
table(
    ['Parameter','Front','Rear'],
    [
        ['Spring rate k','18 500 N/m','42 000 N/m'],
        ['Damping c','900 N·s/m','1 800 N·s/m'],
        ['Natural frequency','fn = (1/2π)√(k/m_front)','fn = (1/2π)√(k/m_rear)'],
    ],
    widths=[2.0,2.2,2.2]
)

h2('How to Read the Graph')
for item in [
    'Both lines near zero: no road input — flat surface, steady speed',
    'Fork compressing (rising) during braking: normal load transfer to front',
    'Rear rising during acceleration: squat — rear spring compresses as load transfers',
    'Oscillations after a bump: check damping ratio — should settle within 1–2 cycles',
    'Fork hitting maximum travel (check against forkTravel parameter): bottoming out — increase spring rate',
    'Rear travel maxed out under heavy braking: too soft rear spring — increase rear spring rate',
]:
    bullet(item)

h2('Damping Ratio Reference')
table(
    ['Damping ratio ζ','Behaviour','Typical use'],
    [
        ['ζ = c / (2√(k·m))','Formula','—'],
        ['< 0.3','Underdamped — many oscillations after bump','Off-road / compliant'],
        ['0.3 – 0.5','Slight oscillation — good road feel','Street bikes'],
        ['0.5 – 0.7','Well-controlled — quick settling','Sport bikes'],
        ['> 0.8','Overdamped — harsh, tyre loses contact on bumps','Track / race'],
    ],
    widths=[2.2,2.5,1.8]
)
rd_box('Spring rate selection: record the peak fork travel during a known brake event (0.8 g). '
       'Optimal spring rate ensures travel stays within 80% of total fork travel. '
       'k_min = F_dynamic / (0.8 × max_travel).')
lim('The suspension model is linear (constant k and c). Real suspensions have progressive spring '
    'rates and velocity-sensitive damping. The Phase 2 backend implements a more complete model.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL H — THERMAL
# ══════════════════════════════════════════════════════════════════════════════
h1('11. Graph Panel H — M5 Tyre Thermal & Wear')

h2('What This Graph Shows')
para(
    'Three side-by-side charts (updated every 150 ms) displaying the M5 tyre thermal model outputs: '
    'temperature (tread and carcass), wear level, and inflation pressure for front and rear tyres. '
    'This panel is only meaningful with the Python backend (M5 thermal model is NOT in C++ Phase 1 engine).'
)

h2('Chart 1: Temperature [°C]')
table(
    ['Channel','Colour','Description'],
    [
        ['Front T_tread','Orange #f97316','Tread surface temperature — most critical for grip'],
        ['Rear T_tread','Cyan #22d3ee dashed [5,2]','Rear tread surface temperature'],
        ['Front T_carcass','Light orange #fb923c dashed [2,3]','Carcass (structural) temperature'],
        ['Rear T_carcass','Light cyan #67e8f9 dashed [2,3]','Rear carcass temperature'],
    ],
    widths=[1.8,2.0,2.8]
)
para('Y-axis range: 0 – 200°C')

h2('Physics — M5 Thermal Model')
code('dT_tread/dt  = (Q_gen − Q_loss_tread) / (m_tread × cp)')
code('dT_carcass/dt = (T_tread − T_carcass) / τ_carcass')
code('Q_gen  = |Fx·vx_slip| + |Fy·vy_slip|    (frictional heat from slip)')
code('Q_loss = (T_tread − T_ambient) / R_thermal')
para('The tyre grip scales with temperature via a Gaussian kernel:')
code('μ_T = μ_base × [1 − (1−cold_grip) × exp(−(T−T_opt)²/(2·T_width²))]')
table(
    ['Parameter','Value','Meaning'],
    [
        ['T_optimal','85°C','Peak grip temperature — Gaussian centre'],
        ['T_width','25°C','Temperature window half-width (68% of peak within ±25°C)'],
        ['T_cold_grip','0.65','Grip fraction at cold temperature (65% of peak)'],
        ['τ_tread','30 s','Tread thermal time constant'],
        ['τ_carcass','120 s','Carcass couples to tread slowly (4× slower)'],
        ['heat_scale','4×10⁻³ °C/J','Tread temperature rise per Joule of slip energy'],
        ['T_ambient','20°C','Ambient air temperature'],
    ],
    widths=[1.8,1.2,3.5]
)

h2('Chart 2: Wear [%]')
table(
    ['Channel','Colour','Range'],
    [
        ['Front wear','Amber #f59e0b','0% = new tyre, 100% = completely worn (bald)'],
        ['Rear wear','Purple #818cf8 dashed [5,2]','Rear typically wears faster under drive torque'],
    ],
    widths=[1.5,2.0,3.0]
)
code('dW/dt = k_wear × (Fx²+Fy²)^(wear_exp/2) / Fz')
code('k_wear = 1.8×10⁻⁸,   wear_exp = 1.5')
code('Grip loss from wear:  μ_wear = μ × (1 − grip_wear_max × W)     W∈[0,1]')
para('grip_wear_max = 0.30: at W=1 (bald tyre), grip drops by 30% maximum.')

h2('Chart 3: Inflation Pressure [kPa]')
table(
    ['Channel','Colour','Normal range'],
    [
        ['Front pressure','Green #4ade80','180–360 kPa (chart range)'],
        ['Rear pressure','Rose #f43f5e dashed [5,2]','p_cold = 230 kPa, p_optimal = 240 kPa'],
    ],
    widths=[1.5,2.0,3.0]
)
para('Pressure rises with temperature (ideal gas law analogy):')
code('p(T) = p_cold × (1 + β × (T_tread − T_ambient))')
para('β ≈ 0.003 /°C — a 65°C tread temperature rise adds ≈ 230 × 0.003 × 65 ≈ 45 kPa.')

h2('How to Read the Graph')
for item in [
    'Temperature: both lines should reach 80–90°C for optimal grip — if cold, the grip scalar drops to 65%',
    'Temperature: above 110°C indicates overheating — grip begins to decrease again',
    'Wear: rear wear rate exceeds front in most scenarios (drive torque creates more slip energy)',
    'Wear > 50%: visible grip reduction — consider extending the compound coefficients',
    'Pressure: rising during warm-up is normal — stabilises when tyre reaches thermal equilibrium',
    'Pressure diverging from front/rear: unequal load distribution or different temperatures',
]:
    bullet(item)

rd_box('Tyre compound comparison: run identical scenarios with different heat_scale and k_wear values '
       '(edit the YAML compound file). Compare the temperature vs lap-time trade-off to replicate '
       'real tyre engineering decisions (soft = fast but wears; hard = slower but durable).')
rd_box('Cold-tyre start analysis: reset simulation with TireState(T_tread=20, T_carcass=20) in the '
       'YAML. Watch the first several seconds of grip deficit — models out-lap behaviour.')
lim('The M5 model is Python-only. C++ Phase 1 backend does NOT transmit thermal channels '
    '(the chart will show flat zero lines). Only Phase 2/FullMF backends include partial thermal.')
lim('Pressure model is empirical (linear with temperature). Real tyre pressure also depends on '
    'flexing frequency and carcass construction — not modelled.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL I — PHASE 1
# ══════════════════════════════════════════════════════════════════════════════
h1('12. Graph Panel I — C++ 13-DOF Engine Diagnostics')
para('Visible ONLY when the C++ 13-DOF backend is active (engine field starts with "C++").')

h2('Chart 1: Dynamic Trail [mm]')
table(
    ['Element','Detail'],
    [
        ['Y axis','Effective trail [mm], range 50–130 mm'],
        ['Channel','effective_trail × 1000  —  orange #f97316'],
    ],
    widths=[1.8,4.6]
)
para('The C++ engine computes DYNAMIC trail from the Cossalter kinematics model:')
code('trail_eff = (R_f·cos(ε) − fork_offset) / sin(ε)  +  Δtrail_steering(δ)')
para('Static trail is purely geometric. Dynamic trail changes with steer angle δ and fork compression '
     '(computed at each 1 kHz step). This is NOT available in the Python backend.')

h2('Chart 2: Dynamic Wheelbase [m]')
table(
    ['Element','Detail'],
    [
        ['Y axis','Effective wheelbase [m], range 1.30–1.42'],
        ['Channel','effective_wheelbase  —  blue #3b82f6'],
    ],
    widths=[1.8,4.6]
)
para('Wheelbase changes with fork compression and steering angle in a true kinematic model. '
     'A 30 mm fork dive shortens the wheelbase by approximately 11 mm (for 24° rake).')

h2('Chart 3: Roll Rate [°/s]')
table(
    ['Element','Detail'],
    [
        ['Y axis','Roll rate [°/s], range −60 to +60'],
        ['Channel','roll_rate  —  purple #a855f7'],
    ],
    widths=[1.8,4.6]
)

h2('R&D Value of Dynamic Trail/Wheelbase')
rd_box('Anti-dive analysis: during a 0.8 g brake event, plot the dynamic trail reduction vs fork compression. '
       'The sensitivity d(trail)/d(fork_travel) is the anti-dive geometric factor. '
       'Compare to the Chassis Workbench AD% calculation.')
rd_box('Steering feedback prediction: larger effective trail → higher self-aligning torque → more '
       'steering feedback force. Dynamic trail explains why braking into a corner "lightens" steering feel.')
lim('Dynamic trail and wheelbase are C++ 13-DOF outputs only. Python simulation uses fixed static trail. '
    'Always use C++ backend when studying steering dynamics.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL J — PHASE 2
# ══════════════════════════════════════════════════════════════════════════════
h1('13. Graph Panel J — Contact Patch & LuGre (Phase 2)')
para('Visible ONLY when Phase 2 or FullMF C++ backend is active.')

h2('Chart 1: Contact Patch Area [cm²]')
table(
    ['Channel','Colour','Description'],
    [
        ['Front area','Cyan #38bdf8','Hertzian contact ellipse area — front tyre [cm²]'],
        ['Rear area','Green #4ade80 dashed [4,2]','Hertzian contact ellipse area — rear tyre [cm²]'],
    ],
    widths=[1.5,1.8,3.3]
)
para('Contact patch area from Hertz contact theory:')
code('A ≈ π × a × b    where a, b = semi-axes of the contact ellipse')
code('a = (3·Fz·R_eff / (4·E*))^(1/3)       (Hertz formula, simplified)')
code('Area scales approximately as Fz^(2/3) for a spherical tyre profile')
para('Y-axis range: 0–60 cm². Normal range: 20–40 cm² at typical road loads.')

h2('Chart 2: LuGre Sliding Fraction [0–1]')
table(
    ['Channel','Colour','Description'],
    [
        ['Front sliding fraction','Red #f87171','0 = full stick (no sliding in patch), 1 = full slide'],
        ['Rear sliding fraction','Orange #fb923c dashed [4,2]','Same for rear'],
    ],
    widths=[1.8,1.8,3.0]
)
para('The LuGre bristle model divides the contact patch into sticking and sliding zones:')
code('dz/dt = v_slip − (|v_slip|/g(v)) × z      [bristle state ODE]')
code('F_LuGre = σ₀·z + σ₁·ż + σ₂·v_slip        [force from bristle deformation]')
code('Sliding fraction = proportion of patch elements with z at saturation')

h2('Chart 3: Lateral Force Fy [N]')
table(
    ['Channel','Colour'],
    [
        ['Fy_front [N]','Purple #c084fc'],
        ['Fy_rear [N]','Amber #fbbf24 dashed [4,2]'],
    ],
    widths=[2.0,4.5]
)

h2('Live Stat Pills (6 values)')
table(
    ['Pill','Symbol','Unit'],
    [
        ['A_f','Front contact area','cm²'],
        ['A_r','Rear contact area','cm²'],
        ['slip_f','Front sliding fraction','%'],
        ['slip_r','Rear sliding fraction','%'],
        ['Fy_f','Front lateral force','N'],
        ['Fy_r','Rear lateral force','N'],
    ],
    widths=[1.0,2.5,1.0]
)

h2('R&D Applications')
rd_box('Tyre construction parameter study: compare contact area between different tyre widths. '
       'Wider tyre → larger A at same Fz → lower average contact pressure → different wear profile.')
rd_box('LuGre sliding fraction as instability predictor: when sliding fraction approaches 1.0, '
       'the tyre is moments away from full sliding. This precedes the Mz reversal visible in Panel E.')
lim('The LuGre model adds significant computation cost. At 1 kHz physics, this limits the Phase 2 '
    'backend to approximately 50× real-time vs 700× for Phase 1.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PANEL K — COEFF FITTER
# ══════════════════════════════════════════════════════════════════════════════
h1('14. Graph Panel K — PINN Coefficient Fitter')
para(
    'Visible ONLY when the C++ backend has active coefficient fitting (cf_updates > 0). '
    'This panel shows the real-time adaptation of Pacejka coefficients by a neural observer — '
    'the PINN coefficient fitter (Phase 3.1).'
)

h2('What Coefficient Fitting Does')
para(
    'The coefficient fitter observes actual tyre forces (measured from the simulation) and adjusts '
    'six Pacejka scale factors to minimise the error between the analytical Pacejka model and the '
    'actual forces. This is real-time system identification — similar to adaptive control.'
)

h2('Six Coefficient Gauge Bars')
table(
    ['Coefficient','Symbol','Physical meaning','Nominal','Colour'],
    [
        ['cf_mu_y','μ_y','Lateral grip peak (scales Fy_max)','1.0','Cyan'],
        ['cf_p_Ky1','K_α','Cornering stiffness (initial slope of Fy vs α)','1.0','Lime'],
        ['cf_E_Fy','E_y','Curvature factor (shape of Fy peak)','1.0','Purple'],
        ['cf_mu_x','μ_x','Longitudinal grip peak (scales Fx_max)','1.0','Orange'],
        ['cf_K_x','K_κ','Longitudinal slip stiffness (initial slope of Fx vs κ)','1.0','Amber'],
        ['cf_tp0','t_p','Pneumatic trail (scales Mz magnitude)','1.0','Rose'],
    ],
    widths=[1.3,0.8,2.5,0.9,0.9]
)
para('All bars are centred at the nominal value of 1.0. A white marker shows the nominal position. '
     'The bar fills to the current estimated value. Red/amber fill indicates the coefficient has '
     'dropped significantly below nominal (> 25% from nominal range).')

h2('Two Trend Charts')
table(
    ['Chart','Y axis','Channels'],
    [
        ['Grip Scales vs Time','Scale factor [0.45–1.15]','μ_y (cyan solid), μ_x (orange dashed)'],
        ['Stiffness & Trail vs Time','Scale factor [0.3–1.6]','K_α (lime), K_κ (amber dashed), t_p (rose dashed)'],
    ],
    widths=[2.0,2.0,2.5]
)
para('A horizontal reference line at 1.0 (nominal) is drawn on both charts.')

h2('How to Read the Panel')
for item in [
    'All coefficients at 1.0: the Pacejka analytical model matches the observed forces perfectly',
    'μ_y or μ_x dropping below 0.7: the tyre has lost significant grip — road surface has changed or tyre is worn',
    'K_α rising above 1.2: tyre is stiffer than nominal — could indicate a colder or over-inflated tyre',
    'tp dropping below 0.7: less pneumatic trail — steering feels lighter than expected',
    'Coefficients oscillating rapidly: the fitter is unstable — check learning rate and noise level',
]:
    bullet(item)

h2('R&D Value')
rd_box('Surface change detection: switch road condition mid-run (wet → dry). Watch how quickly '
       'μ_y and μ_x adapt to the new surface. Response time = convergence rate of the fitter.')
rd_box('Tyre wear progression: run a long simulation. μ_y should track downward as wear increases '
       '(combined effect of wear grip loss from the M5 model). Compare to Panel H wear percentage.')
lim('The coefficient fitter (Phase 3.1) is an experimental feature. Convergence is not guaranteed '
    'under all conditions. Oscillating estimates indicate a poorly excited regime — apply more varied inputs.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# TAB 2 — PARAMETERS
# ══════════════════════════════════════════════════════════════════════════════
h1('15. Tab 2 — Parameters Workspace')
para(
    'The Parameters tab contains all editable simulation parameters grouped into three sections. '
    'Changes do NOT apply until the Apply button is clicked — pressing Apply sends a "set_many" '
    'command followed by a "reset" command to the backend.'
)

h2('Group 1 — Motorcycle Geometry')
table(
    ['Parameter','Unit','Min','Max','Effect'],
    [
        ['Wheelbase','m','0.8','2.0','Primary stability metric. Longer = more stable, wider turning circle.'],
        ['Front wheel diameter','m','0.4','0.8','Sets front axle height. Affects rake angle geometry.'],
        ['Rear wheel diameter','m','0.4','0.8','Sets rear axle height. Affects anti-squat IC.'],
        ['Front tyre width','m','0.08','0.22','Contact patch width. Wider = higher Fz capacity.'],
        ['Rear tyre width','m','0.10','0.28','Same. Rear usually wider for drive load.'],
        ['Rake (°)','°','10','40','Steering axis tilt. High rake = stable/heavy; low = agile/light.'],
        ['Trail','m','0.03','0.20','Steering self-centre tendency. More trail = more stable.'],
        ['CoG height','m','0.2','1.2','Higher = more weight transfer, more pitch sensitivity.'],
    ],
    widths=[1.8,0.5,0.5,0.5,3.2]
)

h2('Group 2 — Mass & Brake Model')
table(
    ['Parameter','Unit','Effect'],
    [
        ['Total mass','kg','Total vehicle + rider mass. Scales all inertial forces.'],
        ['Front weight fraction','—','Static front axle load = fraction × M × g. 0.46 = 46% front.'],
        ['Engine torque max','N·m','Peak torque at throttle=1.0. CBR600RR: 80 N·m.'],
        ['Brake torque max','N·m','Peak brake torque at brake=1.0. Scales Fx during braking.'],
        ['Front spring rate','N/m','Front suspension spring stiffness. Higher = stiffer fork.'],
        ['Rear spring rate','N/m','Rear suspension spring stiffness.'],
        ['Front damping','N·s/m','Front damper coefficient. See damping ratio formula.'],
        ['Rear damping','N·s/m','Rear damper coefficient.'],
        ['Rider mass','kg','Separate from total mass — used for lean arm calculations.'],
        ['Lean arm','m/rad','Rider torque arm for lean command. Typical 0.55 m.'],
    ],
    widths=[2.0,0.6,3.9]
)

h2('Group 3 — Track & Disturbance Inputs')
table(
    ['Parameter','Unit','Effect'],
    [
        ['Track length','m','Total circuit distance before reset (for curved/track modes).'],
        ['Corner radius','m','Radius of the curved section (curved track mode only).'],
        ['Bump height','m','Road bump amplitude in "Bumps" and "Mixed" track modes.'],
        ['Pothole depth','m','Pothole depth in "Potholes" and "Mixed" track modes.'],
        ['Friction coefficient','—','Global μ scale applied to ALL Pacejka forces. Road surface setter.'],
        ['Straight length','m','Straight section length before the corner (curved mode).'],
        ['Corner bank angle','rad','Lateral road slope in the corner section.'],
        ['Reference speed','m/s','Used in Analysis tab calculations (not sent to backend directly).'],
    ],
    widths=[2.0,0.6,3.9]
)

h2('Track Modes')
table(
    ['Mode','Description','Best for'],
    [
        ['Flat','Perfectly flat road, constant μ','Default — baseline all tests on flat'],
        ['Curved (M4)','Straight + corner + straight circuit','Cornering dynamics, anti-squat under acceleration'],
        ['Bumps','Flat road with sinusoidal bumps at bumpHeight amplitude','Suspension tuning — watch travel and Fz oscillation'],
        ['Potholes','Flat road with discrete sharp potholes','Impact harshness — peak Fz and fork travel events'],
        ['Mixed','Bumps + potholes combined','Real-world road simulation'],
    ],
    widths=[1.2,2.8,2.5]
)

tip('Always click Apply then wait 1–2 seconds for the backend to reset and stabilise '
    'before recording data for analysis. The first 2 seconds after reset may show transients.')
rd_box('Parameter sweep study: systematically vary one parameter (e.g. CoG height from 0.50 to 0.70 m '
       'in 0.05 m steps), record the peak values of roll angle and Fz_front in each run. '
       'Build a sensitivity table: ∂(peak_Fz_front) / ∂(h_cg).')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# TAB 3 — ANALYSIS
# ══════════════════════════════════════════════════════════════════════════════
h1('16. Tab 3 — Analysis Workspace')
para(
    'The Analysis tab computes ANALYTICAL (linearised) predictions from the current parameter set '
    'WITHOUT running the simulation. It provides instant engineering estimates — useful before '
    'running a full simulation. All calculations use closed-form equations.'
)

h2('Braking Performance Metrics')
table(
    ['Metric','Formula','Notes'],
    [
        ['Stopping distance [m]','v² / (2 × a_eff)','At current reference speed and effective deceleration'],
        ['Stopping time [s]','v / a_eff','—'],
        ['Effective decel [m/s²]','min(μ × g, τ_brake / (m × R_wheel))','Min of friction limit and brake torque limit'],
    ],
    widths=[2.0,2.5,2.0]
)

h2('Suspension Natural Frequencies')
code('f_front = (1/2π) × √(k_front / (M × front_fraction))')
code('f_rear  = (1/2π) × √(k_rear  / (M × (1 − front_fraction)))')
para('Target: front 1.3–1.7 Hz, rear 1.2–1.6 Hz. Front should be slightly higher than rear.')

h2('Stability Mode Frequencies')
table(
    ['Mode','Formula','Target range','Physical meaning'],
    [
        ['Wobble [Hz]','√((Cα·t + m_f·v²·t/L) / I_steer) / 2π   Cα ≈ 10·Fz_static','6–10 Hz','Steering oscillation — fork weaving side to side'],
        ['Weave [Hz]','v·√(12·h_cg) / (2π·L²)','2–4 Hz','Coupled roll+yaw oscillation'],
        ['Capsize speed [m/s]','√(g·t·L / (h_cg·front_fraction))','> 15 m/s','Speed below which bike falls over without rider'],
    ],
    widths=[1.3,2.8,1.3,2.1]
)
note('These are LINEARISED predictions valid near the equilibrium operating point. '
     'Large-amplitude behaviour (e.g. during a spin-out) is not captured.')

h2('Disturbance Indices')
table(
    ['Index','Formula','Interpretation'],
    [
        ['Bump acceleration [m/s²]','v²/r_bump + 120 × h_bump','Higher = rougher bump impact felt by chassis'],
        ['Pothole severity','18 × v × pothole_depth','Higher = worse — speed amplifies pothole impact'],
    ],
    widths=[2.0,2.5,2.0]
)

h2('Graph 1: Dynamic Fz — Weight Transfer Preview')
para(
    'An SVG line chart showing predicted front and rear axle loads across the deceleration range 0–1.0 g. '
    'X axis: deceleration [g]. Y axis: normal load Fz [N]. '
    'Lime = Fz_front (rises under braking), cyan dashed = Fz_rear (drops). '
    'This is the STATIC weight transfer prediction — no suspension dynamics.'
)
code('Fz_front(a) = Fz_static_f + M × a × h_cg / L')
code('Fz_rear(a)  = Fz_static_r − M × a × h_cg / L')
para('Read: at 1.0 g decel, if Fz_rear reaches zero → stoppie (rear lift). '
     'The crossing point of Fz_rear with the zero axis gives the stoppie threshold.')

h2('Graph 2: Stopping Distance vs Speed')
para(
    'A brown curve showing stopping distance as a function of initial speed (0 to 30 m/s). '
    'X axis: initial speed [m/s]. Y axis: stopping distance [m].'
)
code('d_stop(v) = v² / (2 × a_effective)')
para('This is a pure energy argument — the quadratic relationship means doubling speed quadruples stopping distance. '
     'A landmark: at 20 m/s (72 km/h) with 0.9 g decel (8.83 m/s²), stopping distance ≈ 22.6 m.')

h2('Stability Frequencies Bar Chart')
para('A visual bar chart comparing wobble, weave, front ride, and rear ride frequencies against '
     'reference values. Bars coloured green (within range) or amber (outside range).')
rd_box('Change CoG height in Parameters tab, then compare the wobble and weave frequency predictions '
       'in the Analysis tab. Increasing CoG height by 50 mm typically raises weave frequency by 0.3–0.5 Hz '
       'and reduces capsize speed by 2–3 m/s. This is a key suspension/geometry interaction.')
rd_box('Stopping distance chart: overlay two curves with different μ values to show the effect of wet '
       'road on braking distance. Present this to non-technical stakeholders as a clear safety argument.')
lim('The Analysis tab uses linearised closed-form models. It does NOT use Pacejka — it uses simplified '
    'constant-Cα approximations. Use the Live tab simulation for accurate force values.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# TAB 4 — ALL GRAPHS
# ══════════════════════════════════════════════════════════════════════════════
h1('17. Tab 4 — All Graphs')
para(
    'The All Graphs tab displays all 11 live time-series charts simultaneously in a 3-column grid '
    '(updated every 180 ms, maximum 800 points per series). '
    'All Y-axes auto-range (except thermal) and all X-axes show time [s]. '
    'This view is ideal for holistic monitoring during a complex test run — every channel is visible at once.'
)

h2('Complete Channel Index (All Graphs Tab)')
table(
    ['Chart #','Title','Channels','Y axis'],
    [
        ['1','Speed & Longitudinal Accel','vx [m/s] (cyan), ax [m/s²] (orange dashed)','Dual axis'],
        ['2','Chassis Angles','roll [°] (red), pitch [°] (amber dashed), yaw_rate [°/s] (blue dashed)','Auto'],
        ['3','Roll Rate & Yaw Rate','roll_rate [°/s] (red), yaw_rate [°/s] (blue dashed)','Auto'],
        ['4','Normal Load Transfer','Fz_front [N] (lime), Fz_rear [N] (cyan dashed)','≥ 0'],
        ['5','Tyre Forces Fx/Fy','Fx_f (amber), Fx_r (orange), Fy_f (purple), Fy_r (purple dashed)','Auto'],
        ['6','Aligning Torque Mz','Mz_front [N·m] (green), Mz_rear [N·m] (purple dashed)','Auto'],
        ['7','Slip Angles α & Ratio κ','α_f [°] (purple), α_r [°] (purple dashed), κ_f (orange), κ_r (amber dashed)','Dual axis'],
        ['8','Steer Angle & Torque','δ [°] (amber), τ [N·m] (lime dashed)','Dual axis'],
        ['9','Suspension Travel','Fork [mm] (amber), Rear [mm] (purple dashed)','Auto'],
        ['10','Tyre Thermal — Front','T_tread [°C] (rose), T_carcass [°C] (orange dashed) left; wear [%] (amber), p [kPa] (cyan) right','0–200°C / 0–100%'],
        ['11','Tyre Thermal — Rear','Same as chart 10 but rear tyre channels','0–200°C / 0–100%'],
    ],
    widths=[0.5,2.0,2.8,1.2]
)

h2('How to Use All Graphs Tab Effectively')
for item in [
    'Run the simulation for at least 30 seconds to build up meaningful history before switching to this tab',
    'Use the expand button (⤢) on individual Live tab panels for the same channel in a larger view',
    'Look for correlations: Fz_front rising (chart 4) should coincide with α_front rising (chart 7)',
    'Chassis oscillations visible in chart 2 should have matching frequency spikes in charts 5 and 7',
    'Export the session CSV after recording for offline analysis in Python/Excel',
]:
    bullet(item)

tip('All Graphs tab is best used as a "recording monitor" while you manipulate controls in '
    'the Live tab — open All Graphs in a second monitor if available.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PHYSICS DEEP DIVE — PACEJKA
# ══════════════════════════════════════════════════════════════════════════════
h1('18. Physics Deep-Dive — Pacejka Magic Formula')

h2('Overview')
para(
    'The Pacejka Magic Formula (MF-6) is the industry-standard tyre model used in this simulation. '
    'It was developed by Prof. Hans Pacejka (TU Delft) and validated against extensive tyre rig measurements. '
    'It computes Fx, Fy, and Mz as analytical functions of slip angle (α), slip ratio (κ), '
    'normal load (Fz), and camber angle (γ).'
)

h2('Pure-Slip Lateral Force Fy (the "Magic Formula")')
code('D     = μ × Fz                              [peak force, N]')
code('B     = k_alpha × Fz / (C_pac × D + ε)     [stiffness factor]')
code('φ     = B·α − E·(B·α − atan(B·α))          [shape function input]')
code('Fy_0  = D · sin(C_pac · atan(φ))            [pure-slip lateral force]')
code('Fy_0  += k_gamma × Fz × γ                  [add camber thrust]')
code('Fy_0  = clamp(Fy_0, −D, +D)                [physical limit]')
table(
    ['Coefficient','Symbol','Default','Physical meaning'],
    [
        ['Friction','μ','0.90','Peak tyre–road friction coefficient. Scales everything.'],
        ['Cornering stiffness','k_alpha (BCD)','12.0','Initial slope of Fy vs α [N/rad per N of Fz]. Higher = stiffer tyre.'],
        ['Camber stiffness','k_gamma','0.90','Lateral force per unit camber [N/N/rad]. Important for motorcycles.'],
        ['Shape factor','C_pac','1.80','Controls the width of the Fy curve. C=1.8 gives typical motorcycle shape.'],
        ['Curvature factor','E_pac','0.00','Controls the peak sharpness. E=0 → symmetric peak.'],
    ],
    widths=[1.8,1.3,1.0,2.9]
)

h2('Pure-Slip Longitudinal Force Fx')
code('Fx_0 = μ × Fz × tanh(Kx × κ)              [simplified MF longitudinal]')
code('Kx = 12.0   [longitudinal slip stiffness — initial slope of Fx vs κ]')
para('Note: tanh approximates the full MF sine formula for |κ| < 0.3, '
     'which covers the normal operating range.')

h2('Combined-Slip Weighting (Pacejka 2012 §4.3.2)')
para(
    'When both α ≠ 0 and κ ≠ 0 simultaneously, the combined-slip G-factors reduce each force. '
    'This is the key interaction: braking while cornering reduces both Fx AND Fy:'
)
code('Gxα = cos(Cxa · atan(Bxa · α))    ← reduces Fx due to lateral slip')
code('Gyκ = cos(Cyk · atan(Byk · κ))    ← reduces Fy due to longitudinal slip')
code('Fx  = Gxα × Fx_0    (always ≤ Fx_0)')
code('Fy  = Gyκ × Fy_0    (always ≤ Fy_0)')
table(
    ['Coefficient','Default','Effect'],
    [
        ['Bxa (Gxα stiffness)','10.0 [1/rad]','Higher = Fx drops faster with increasing α'],
        ['Cxa (Gxα shape)','1.00','Keep ≤ 1 to ensure G ≥ 0 always'],
        ['Byk (Gyκ stiffness)','8.00','Higher = Fy drops faster with increasing κ'],
        ['Cyk (Gyκ shape)','1.00','Keep ≤ 1'],
    ],
    widths=[2.0,1.5,3.0]
)

h2('Aligning Torque Mz')
code('tp   = tp0 · cos(Ct · atan(Bt · α′))       [pneumatic trail]')
code('Mz   = −Fy_combined · tp  +  K_gamma_Mz · γ · Fz   [total aligning torque]')
para('At α=0: tp = tp0 = 30 mm (maximum trail). As α increases to the peak, tp → 0. '
     'Beyond peak: tp < 0 → Mz reversal → the dreaded "sudden front wash" warning.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PHYSICS — THERMAL, WEAR, PRESSURE
# ══════════════════════════════════════════════════════════════════════════════
h1('19. Physics Deep-Dive — M5 Thermal, Wear & Pressure')

h2('Thermal Model — Heat Generation')
code('Q_gen   = |Fx · v_slip_x| + |Fy · v_slip_y|     [W — frictional heat flux]')
code('v_slip  = |R_w · ω − vx|   for longitudinal,   vy_contact for lateral')
para('More slip energy → faster tyre warm-up. This explains why out-laps feel cold: '
     'minimal slip → minimal heat generation.')

h2('Thermal Model — Temperature Dynamics')
code('dT_tread/dt  = heat_scale × Q_gen − (T_tread − T_amb) / τ_tread')
code('dT_carcass/dt = (T_tread − T_carcass) / τ_carcass')
para('τ_tread = 30 s (fast responding surface), τ_carcass = 120 s (slow responding structure). '
     'The tread reaches equilibrium ~5× faster than the carcass.')

h2('Grip vs Temperature — Gaussian Window')
code('μ_T = μ × [1 − (1 − cold_grip) × exp(−(T_tread − T_opt)² / (2 × T_width²))]')
para('This gives:')
table(
    ['Temperature','Grip fraction','Consequence'],
    [
        ['20°C (cold)','65% of peak','New tyres on cold morning — significant grip deficit'],
        ['60°C (warming)','~87% of peak','Transitional — grip improving rapidly'],
        ['85°C (optimal)','100% of peak','Full grip available'],
        ['110°C (hot)','~87% of peak','Overheating — same as 60°C but approaching'],
        ['130°C+ (very hot)','< 80%','Severe overheating — tyre degrading rapidly'],
    ],
    widths=[1.8,1.5,3.2]
)

h2('Wear Model')
code('dW/dt = k_wear × (Fx² + Fy²)^(wear_exp/2) / Fz')
code('Grip loss: μ_wear = μ × (1 − grip_wear_max × W),   grip_wear_max = 0.30')
para('At W=0.5 (50% worn): grip = μ × (1 − 0.30 × 0.5) = 0.85μ (15% grip loss). '
     'At W=1.0 (bald): grip = μ × 0.70 (30% loss).')

h2('Pressure Model')
code('p(T) = p_cold + β × (T_tread − T_ambient)    β ≈ 0.003 per °C')
para('p_cold = 230 kPa, p_optimal = 240 kPa. A 33°C rise from cold adds ~10 kPa.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# PHYSICS — CHASSIS DYNAMICS
# ══════════════════════════════════════════════════════════════════════════════
h1('20. Physics Deep-Dive — Chassis Dynamics & Stability')

h2('The 13-DOF Motorcycle Model (C++ Backend)')
table(
    ['DOF','Variable','Description'],
    [
        ['1','x, y, z','Position of chassis CoG in world frame'],
        ['2','Roll φ','Lean angle'],
        ['3','Pitch θ','Front/rear tilt'],
        ['4','Yaw ψ','Heading direction'],
        ['5','Steer δ','Handlebar angle'],
        ['6–8','ω_front, ω_rear','Wheel spin rates'],
        ['9–10','z_front_susp, z_rear_susp','Suspension deflections'],
        ['11','Rider lean (upper body)','Rider lean mass relative to chassis'],
        ['12–13','Tyre α_prime_f, α_prime_r','Relaxed slip angles (state variables)'],
    ],
    widths=[0.8,2.0,3.7]
)

h2('Weight Transfer')
code('ΔFz = M · a · h_cg / L      (braking: positive → front; acceleration: positive → rear)')

h2('Wheelie Threshold')
code('a_wheelie = g · (L − x_cg) / h_cg')
para('At default geometry: a_wheelie = 9.81 × (1.37 × 0.54) / 0.58 ≈ 12.4 m/s² ≈ 1.26 g. '
     'Application of throttle must stay below this to avoid front lift.')

h2('Stoppie Threshold')
code('a_stoppie = g · x_cg / h_cg')
para('At default: a_stoppie = 9.81 × (1.37 × 0.46) / 0.58 ≈ 10.6 m/s² ≈ 1.08 g.')

h2('Stability Mode Summary')
table(
    ['Mode','Frequency','Nature','Dangerous if'],
    [
        ['Capsize','0 Hz (divergent)','Unstable lean — bike falls without rider','Below capsize speed (~4 m/s); stable at road speed'],
        ['Weave','2–4 Hz','Coupled roll + yaw oscillation','Amplitude grows at high speed or low damping'],
        ['Wobble','6–10 Hz','Steering oscillation (fork weave)','Underdamped steering head; worn steering head bearings'],
    ],
    widths=[1.2,1.3,2.5,2.0]
)

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# BACKENDS
# ══════════════════════════════════════════════════════════════════════════════
h1('21. Backend Engines — Python vs C++ vs PINN')
table(
    ['Feature','Python Simulation','C++ Phase 1','C++ Phase 2/FullMF','PINN'],
    [
        ['Speed','~1× real-time','~700× real-time','~50× real-time','~200× real-time'],
        ['Tyre model','Pacejka MF + M5 thermal','Pacejka MF-6','FullMF + LuGre + Hertz','Neural net (ONNX)'],
        ['Thermal/Wear','YES (M5)','NO','Partial','NO'],
        ['Dynamic trail','NO (static)','YES (Cossalter kinematics)','YES','YES'],
        ['Contact patch','Point contact','Point contact','Hertzian ellipse','Point contact'],
        ['Coefficient fitter','NO','Optional','Optional','YES (live adaptation)'],
        ['Road conditions','All 4','All 4','All 4','All 4'],
        ['Best use','Thermal/wear research','Speed, geometry studies','Contact/LuGre research','Adaptive tyre ID'],
    ],
    widths=[1.8,1.4,1.4,1.6,1.2]
)
note('The Python backend runs physics at 1 kHz regardless of real-time ratio. '
     'The C++ backend runs the same 1 kHz loop but the integrator is 700× faster, '
     'meaning the backend can simulate 700 seconds of physics per real second. '
     'The WS output is still throttled to 50 Hz regardless.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# ROAD CONDITIONS
# ══════════════════════════════════════════════════════════════════════════════
h1('22. Road Conditions & Track Modes')
table(
    ['Road','μ','Banking','Gradient','Notes'],
    [
        ['Dry Asphalt','1.0','0','0','Default reference surface'],
        ['Wet Asphalt','0.60','0','0','Heavy rain — 40% grip reduction'],
        ['Race Track (Dry)','1.10','0.04 rad','0','High-grip compound + banked corner'],
        ['Mountain Road','0.90','0.02 rad','0.08 (8%)','Uphill gradient with slight banking'],
    ],
    widths=[1.8,0.6,1.0,1.2,2.4]
)
para('When running the C++ backend, a YAML road file is loaded from scenarios/roads/. '
     'Each YAML defines the surface profile, μ schedule, and geometry along the track.')

rd_box('Wet-to-dry transition study: start simulation on wet road, mid-run switch road to dry '
       'via the Parameters tab and Apply. Watch the friction circle expand as μ changes. '
       'Measure the adaptation delay if the PINN coefficient fitter is active.')
lim('The Python FlatPlane road model is truly flat and uniform. Corner banking and gradient '
    'are only available via the C++ YAML road files. The friction coefficient is spatially uniform '
    'in all current road models — no puddles, oil patches, or tyre tracks.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# RECORDING
# ══════════════════════════════════════════════════════════════════════════════
h1('23. Session Recording & Data Export')

h2('Recording a Session')
for i,item in enumerate([
    'Click ⏺ REC in the top bar — the button turns red and shows "⏹ STOP". The buffer is cleared.',
    'Run the simulation scenario you want to capture.',
    'Click ⏹ STOP. The session is saved to disk and a toast notification shows the save path.',
    'The save directory appears in /tmp/motorcycle_dashboard_logs/ or the configured output path.',
],1):
    bullet(item, bold=f'Step {i}: ')

h2('Exported Files')
table(
    ['File','Format','Contents'],
    [
        ['session_YYYYMMDD_HHMMSS.csv','CSV','One row per WS frame (~50 Hz). All ~50 channels as columns.'],
        ['session_YYYYMMDD_HHMMSS.json','JSON','Identical data in structured JSON format.'],
        ['launcher.log','Text','Backend start/stop events and errors.'],
        ['simulator.log','Text','Backend stdout/stderr — physics error messages.'],
    ],
    widths=[2.5,1.0,3.0]
)

h2('All Telemetry Channels (50 Hz CSV columns)')
table(
    ['Column','Unit','Description'],
    [
        ['t','s','Simulation time'],
        ['roll','rad','Chassis roll (lean) angle'],
        ['pitch','rad','Chassis pitch angle'],
        ['yaw','rad','Chassis yaw (heading) angle'],
        ['roll_rate','rad/s','Roll angular velocity'],
        ['yaw_rate','rad/s','Yaw angular velocity'],
        ['vx','m/s','Longitudinal velocity'],
        ['vy','m/s','Lateral velocity'],
        ['ax','m/s²','Longitudinal acceleration'],
        ['ay','m/s²','Lateral acceleration'],
        ['f_alpha','rad','Front slip angle (kinematic)'],
        ['f_kappa','—','Front slip ratio'],
        ['f_Fx','N','Front longitudinal force'],
        ['f_Fy','N','Front lateral force'],
        ['f_Fz','N','Front normal load'],
        ['f_Mz','N·m','Front aligning torque'],
        ['f_gamma','rad','Front camber angle'],
        ['r_alpha, r_kappa, r_Fx, r_Fy, r_Fz, r_Mz, r_gamma','—','Same for rear tyre'],
        ['f_travel, r_travel','m','Fork and rear suspension travel'],
        ['steer_angle','rad','Handlebar steering angle'],
        ['steer_torque','N·m','Applied steer torque (rider input)'],
        ['throttle, brake_f, brake_r','—','Rider control inputs [0–1]'],
        ['f_T_tread, f_T_carcass','°C','Front tyre temperatures'],
        ['f_wear','—','Front wear [0–1]'],
        ['f_pressure','kPa','Front inflation pressure'],
        ['r_T_tread, r_T_carcass, r_wear, r_pressure','—','Same for rear'],
        ['road_mu_f, road_mu_r','—','Local road friction at front/rear contact point'],
        ['engine','string','Backend identifier string'],
    ],
    widths=[2.5,0.8,3.2]
)

rd_box('Post-processing recipe (Python): load CSV with pandas, compute the friction circle '
       'analytically as mu_x = f_Fx/f_Fz, mu_y = f_Fy/f_Fz, plot as scatter. '
       'Compare to the live friction circle dashboard to validate the real-time display.')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# LIMITATIONS
# ══════════════════════════════════════════════════════════════════════════════
h1('24. System Limitations')

h2('Physics Model Limitations')
table(
    ['Limitation','Affected panels','Workaround / Future work'],
    [
        ['Point-contact tyre — no contact patch length dynamics','All force panels (B, C, D)','Use Phase 2 backend for Hertzian contact'],
        ['Python tyre model: no turn-slip Fy component','Panel B (Fy at low speed)','Use C++ FullMF backend'],
        ['Static mass distribution — no rider body multi-segment model','Panel A (Fz)','Rider mass treated as single-point CoG at rider_lean_arm'],
        ['Linear spring/damper — no progressive spring rate or velocity-sensitive damping','Panel G (Suspension)','Phase 2 backend has extended suspension model'],
        ['M5 thermal model: Python only — not available in C++ Phase 1','Panel H (Thermal)','Use Python backend for thermal studies'],
        ['Flat road only (Python backend) — no 3D road surface','All panels','Use C++ backend with YAML road files'],
        ['No aerodynamic forces in Python backend','Panel A (Fz at high speed)','C++ backend includes basic aero drag'],
        ['No tyre belt dynamics (faster than σ/vx transients)','Panel D (Slip)','Not modelled — use relaxation length as approximation'],
        ['Friction circle shows front tyre only','Panel C','Rear tyre budget not directly visible — add second circle manually from CSV data'],
        ['No gyroscopic effects from wheels in Python backend','Panel F (Chassis)','C++ FullMF backend includes gyroscopic Mz from wheel spin'],
    ],
    widths=[2.5,1.5,2.5]
)

h2('Software Limitations')
for item in [
    'WebSocket at 50 Hz: all graphs have 20 ms data latency vs the 1 ms physics timestep',
    'Chart buffer limited to 800 points (All Graphs) and 140 ms update rate (Live panels) — high-frequency events (> 25 Hz) are aliased',
    'Session recording saves to /tmp — files are lost on reboot. Move important sessions to persistent storage immediately',
    'Multiple simultaneous sessions on the same port (8765) are not supported — only one backend at a time',
    'No authentication on WebSocket — anyone on the local network can send commands',
]:
    bullet(item)

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# R&D USE CASES
# ══════════════════════════════════════════════════════════════════════════════
h1('25. R&D Use Cases & Workflow Recipes')

h2('Use Case 1: Suspension Spring Rate Selection')
para('Goal: find the minimum front spring rate that prevents fork bottoming under 0.8 g braking.')
for i,item in enumerate([
    'Set baseline parameters. Track mode: Flat. Apply and run.',
    'Apply 0.8 g brake (brake_f = 0.8 in Control Panel).',
    'Read peak fork travel from Panel G (Suspension Travel).',
    'If peak travel > 80% of total fork length: increase front spring rate by 10% in Parameters tab.',
    'Repeat from step 2 until peak travel ≤ 80%. Record optimal spring rate.',
    'Verify natural frequency in Analysis tab remains ≥ 1.3 Hz.',
],1):
    bullet(item, bold=f'Step {i}: ')

h2('Use Case 2: Tyre Compound Thermal Study')
para('Goal: compare warm-up rate of soft vs medium compound.')
for i,item in enumerate([
    'Edit core/tire/pacejka_coeffs/medium.yaml: copy to soft.yaml, set tau_tread = 15 (half), heat_scale = 8e-3 (double).',
    'Run simulation with medium compound, Python backend, Flat road. Monitor Panel H temperature.',
    'Record time-to-T_optimal (T_tread reaches 85°C) from timestamp t=0.',
    'Swap to soft compound (edit YAML path in moto_sim_main.py). Repeat.',
    'Compare time-to-optimal grip between compounds. Soft: faster warm-up, faster wear.',
],1):
    bullet(item, bold=f'Step {i}: ')

h2('Use Case 3: Corner-Exit Traction Analysis')
para('Goal: find the maximum throttle application that keeps the traction circle inside the friction limit.')
for i,item in enumerate([
    'Set track mode: Curved. Apply and run at constant low throttle through the corner.',
    'Monitor the Friction Circle (Panel C). Note the steady-state μy during cornering.',
    'Gradually increase throttle target speed (CC slider) through the corner.',
    'Watch the yellow dot in the Friction Circle move diagonally toward the top-right.',
    'When the dot approaches the circle boundary: you have found the traction limit.',
    'Record the throttle % and speed. This is your data for anti-squat geometry optimisation.',
],1):
    bullet(item, bold=f'Step {i}: ')

h2('Use Case 4: Wet vs Dry Performance Comparison')
para('Goal: quantify stopping distance difference between dry and wet conditions.')
for i,item in enumerate([
    'Parameters tab: set reference speed to 20 m/s. Note stopping distance from Analysis tab (dry).',
    'Set friction coefficient to 0.6 (wet). Apply. Note stopping distance from Analysis tab (wet).',
    'Live comparison: run both with full brake (brake_f=1.0, brake_r=0.3). Record time to v=0.',
    'Export both sessions as CSV and compute v(t) integral to get actual stopping distance.',
],1):
    bullet(item, bold=f'Step {i}: ')

h2('Use Case 5: Stability Mode Identification (for researchers)')
para('Goal: measure wobble and weave natural frequencies experimentally.')
for i,item in enumerate([
    'Set speed to 20 m/s (cruise control). Wait for steady state.',
    'Apply a brief steer pulse (steer torque = +10 N·m for 0.1 s, then release).',
    'Record the resulting oscillation in Panel F (roll rate and yaw rate channels).',
    'After recording, export CSV. Use Python FFT: scipy.signal.welch on yaw_rate column.',
    'Identify peaks: wobble ≈ 6–10 Hz, weave ≈ 2–4 Hz.',
    'Compare to Analysis tab predictions. Discrepancy = model–reality gap to investigate.',
],1):
    bullet(item, bold=f'Step {i}: ')

doc.add_page_break()

# ══════════════════════════════════════════════════════════════════════════════
# QUICK REFERENCE
# ══════════════════════════════════════════════════════════════════════════════
h1('26. Quick Reference — Symbols, Channels & Target Ranges')

h2('All Physics Symbols Used in Dashboard')
table(
    ['Symbol','Name','Unit','Typical range'],
    [
        ['α','Slip angle','° (rad in backend)','±0° to ±12° (above 8° = near limit)'],
        ['κ','Slip ratio','—','−0.2 to +0.2 normal; |κ|>0.3 = excessive slip'],
        ['γ','Camber angle (tyre lean)','rad','Equals roll angle for solid rear axle'],
        ['φ','Roll angle (lean)','° (rad in backend)','±0° to ±45° (> 30° = aggressive)'],
        ['θ','Pitch angle','° (rad in backend)','±3° normally; > 10° = wheelie/stoppie'],
        ['ψ','Yaw angle','° (rad in backend)','Cumulative heading change'],
        ['ψ̇ (yaw_rate)','Yaw rate','°/s','< 60°/s typical road; > 100°/s = hard corner'],
        ['φ̇ (roll_rate)','Roll rate','°/s','< 30°/s normal; > 60°/s = rapid lean'],
        ['δ','Steer angle','°','−5° to +5° steady state; ±15° max'],
        ['τ','Steer torque','N·m','−30 to +30 N·m rider input'],
        ['Fx','Longitudinal tyre force','N','−1500 to +800 N (braking to drive)'],
        ['Fy','Lateral tyre force','N','−1800 to +1800 N (cornering)'],
        ['Fz','Normal tyre load','N','600–1800 N front; 900–2200 N rear'],
        ['Mz','Aligning torque','N·m','−30 to +30 N·m front tyre'],
        ['μx','Normalised long. force','—','= Fx/Fz; range −1.0 to +1.0'],
        ['μy','Normalised lat. force','—','= Fy/Fz; range −1.0 to +1.0'],
        ['tp','Pneumatic trail','m','0 to 0.030 m (drops to 0 at tyre saturation)'],
        ['σ','Relaxation length','m','0.30 m motorcycle; determines lag in α response'],
        ['T_tread','Tread temperature','°C','20–130°C (optimal 85°C)'],
        ['W','Tyre wear level','—','0 (new) to 1 (bald); > 0.5 = noticeable grip loss'],
        ['p','Inflation pressure','kPa','230–280 kPa operating range'],
        ['vx','Longitudinal velocity','m/s','0 to 80 m/s (0–288 km/h)'],
        ['h_cg','CoG height','m','0.50–0.70 m typical motorcycle'],
        ['L','Wheelbase','m','1.30–1.60 m typical motorcycle'],
    ],
    widths=[1.2,2.0,1.0,2.3]
)

h2('Alert Colour Coding Summary (across all panels)')
table(
    ['Colour','Meaning','Example panels'],
    [
        ['Green / Lime','Nominal / safe / accelerating','Fz_front rising safely, throttle > 50%'],
        ['Cyan','Secondary safe channel','Fz_rear, rear tyre channels'],
        ['Amber / Orange','Warning / approaching limit','FPS < 40, steer torque high, brake active'],
        ['Red','Alert / exceeded limit / braking','Roll > 0.5 rad, Fz_front > 65%, Mz reversal'],
        ['Purple','Lateral / rear tyre channels','Fy_rear, Mz_rear, rear wear'],
        ['Yellow','Current operating point (friction circle)','Friction circle current point'],
        ['Pink ring','Friction limit boundary','Friction circle limit circle'],
    ],
    widths=[1.5,2.5,2.5]
)

h2('Backend-Specific Panel Availability')
table(
    ['Panel','Python','C++ Ph1','C++ Ph2 / FullMF','PINN'],
    [
        ['Normal Load (A)','✓','✓','✓','✓'],
        ['Tyre Forces Fx/Fy (B)','✓','✓','✓','✓'],
        ['Friction Circle (C)','✓','✓','✓','✓'],
        ['Slip Angles (D)','✓','✓','✓','✓'],
        ['Aligning Torque Mz (E)','✓','✓','✓','✓'],
        ['Chassis Angles (F)','✓','✓','✓','✓'],
        ['Suspension Travel (G)','✓','✓','✓','✓'],
        ['Thermal & Wear (H)','✓','✗','Partial','✗'],
        ['C++ Diagnostics (I)','✗','✓','✓','✓'],
        ['Contact Patch & LuGre (J)','✗','✗','✓','✗'],
        ['PINN Coeff Fitter (K)','✗','Optional','Optional','✓'],
    ],
    widths=[2.2,0.8,0.9,1.5,0.9]
)

# ── Save ──────────────────────────────────────────────────────────────────────
out = '/home/dikshant/Desktop/Project vi grade/Startup/moto_sim/Motorcycle_Dashboard_User_Guide.docx'
doc.save(out)
print(f'Saved: {out}')
