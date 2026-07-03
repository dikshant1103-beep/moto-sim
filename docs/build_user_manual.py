"""
build_user_manual.py — generates Motorcycle_Simulator_User_Manual.docx
Run from any directory:
    python3 docs/build_user_manual.py
Output: docs/Motorcycle_Simulator_User_Manual.docx
"""

from pathlib import Path
from docx import Document
from docx.shared import Pt, RGBColor, Inches, Cm
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement
import copy

OUT = Path(__file__).parent / "Motorcycle_Simulator_User_Manual.docx"

# ── Colour palette ────────────────────────────────────────────────────────────
C_TITLE   = RGBColor(0xC4, 0x4B, 0x2D)   # accent red-orange
C_H1      = RGBColor(0x16, 0x5D, 0x76)   # accent teal
C_H2      = RGBColor(0x21, 0x1B, 0x16)   # near-black
C_H3      = RGBColor(0x6F, 0x62, 0x55)   # muted brown
C_BODY    = RGBColor(0x21, 0x1B, 0x16)
C_CODE    = RGBColor(0x16, 0x5D, 0x76)
C_NOTE_BG = RGBColor(0xF0, 0xF9, 0xFF)
C_WARN    = RGBColor(0xEA, 0x58, 0x0C)

doc = Document()

# ── Page margins ──────────────────────────────────────────────────────────────
for section in doc.sections:
    section.top_margin    = Cm(2.0)
    section.bottom_margin = Cm(2.0)
    section.left_margin   = Cm(2.5)
    section.right_margin  = Cm(2.5)

# ── Helper functions ──────────────────────────────────────────────────────────

def set_font(run, name="Calibri", size=11, bold=False, italic=False, color=None):
    run.font.name  = name
    run.font.size  = Pt(size)
    run.font.bold  = bold
    run.font.italic = italic
    if color:
        run.font.color.rgb = color

def para(text, style="Normal", align=WD_ALIGN_PARAGRAPH.LEFT,
         bold=False, size=11, color=C_BODY, italic=False, space_before=0, space_after=6):
    p = doc.add_paragraph(style=style)
    p.alignment = align
    p.paragraph_format.space_before = Pt(space_before)
    p.paragraph_format.space_after  = Pt(space_after)
    run = p.add_run(text)
    set_font(run, size=size, bold=bold, color=color, italic=italic)
    return p

def heading1(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(16)
    p.paragraph_format.space_after  = Pt(4)
    run = p.add_run(text)
    set_font(run, size=16, bold=True, color=C_H1)
    # Add bottom border
    pPr = p._p.get_or_add_pPr()
    pBdr = OxmlElement('w:pBdr')
    bottom = OxmlElement('w:bottom')
    bottom.set(qn('w:val'), 'single')
    bottom.set(qn('w:sz'), '4')
    bottom.set(qn('w:space'), '2')
    bottom.set(qn('w:color'), '165D76')
    pBdr.append(bottom)
    pPr.append(pBdr)
    return p

def heading2(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(10)
    p.paragraph_format.space_after  = Pt(3)
    run = p.add_run(text)
    set_font(run, size=13, bold=True, color=C_H2)
    return p

def heading3(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(6)
    p.paragraph_format.space_after  = Pt(2)
    run = p.add_run(text)
    set_font(run, size=11, bold=True, color=C_H3, italic=True)
    return p

def body(text, space_after=6):
    return para(text, size=11, color=C_BODY, space_after=space_after)

def bullet(text, level=0):
    p = doc.add_paragraph(style="List Bullet")
    p.paragraph_format.space_after  = Pt(3)
    p.paragraph_format.left_indent  = Inches(0.25 + level * 0.25)
    run = p.add_run(text)
    set_font(run, size=10.5, color=C_BODY)
    return p

def numbered(text):
    p = doc.add_paragraph(style="List Number")
    p.paragraph_format.space_after = Pt(3)
    run = p.add_run(text)
    set_font(run, size=10.5, color=C_BODY)
    return p

def code_line(text):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(1)
    p.paragraph_format.space_after  = Pt(1)
    p.paragraph_format.left_indent  = Inches(0.3)
    run = p.add_run(text)
    set_font(run, name="Courier New", size=9.5, color=C_CODE)
    return p

def note_box(text, prefix="NOTE"):
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after  = Pt(4)
    p.paragraph_format.left_indent  = Inches(0.3)
    run1 = p.add_run(f"{prefix}  ")
    set_font(run1, size=10, bold=True,
             color=C_WARN if prefix=="WARNING" else C_H1)
    run2 = p.add_run(text)
    set_font(run2, size=10, color=C_BODY, italic=True)
    return p

def table_2col(rows, header=None, col_widths=(Inches(2.2), Inches(4.0))):
    tbl = doc.add_table(rows=0, cols=2)
    tbl.style = "Table Grid"
    tbl.alignment = WD_TABLE_ALIGNMENT.LEFT
    if header:
        row = tbl.add_row()
        for i, h in enumerate(header):
            c = row.cells[i]
            c.width = col_widths[i]
            run = c.paragraphs[0].add_run(h)
            set_font(run, size=10, bold=True, color=C_H1)
            c.paragraphs[0].paragraph_format.space_after = Pt(2)
    for left, right in rows:
        row = tbl.add_row()
        row.cells[0].width = col_widths[0]
        row.cells[1].width = col_widths[1]
        r0 = row.cells[0].paragraphs[0].add_run(left)
        r1 = row.cells[1].paragraphs[0].add_run(right)
        set_font(r0, size=10, bold=True, color=C_H2)
        set_font(r1, size=10, color=C_BODY)
        row.cells[0].paragraphs[0].paragraph_format.space_after = Pt(2)
        row.cells[1].paragraphs[0].paragraph_format.space_after = Pt(2)
    doc.add_paragraph()   # spacer after table
    return tbl

def table_header_row(rows, headers, col_widths=None):
    ncols = len(headers)
    if col_widths is None:
        col_widths = [Inches(6.0 / ncols)] * ncols
    tbl = doc.add_table(rows=0, cols=ncols)
    tbl.style = "Table Grid"
    tbl.alignment = WD_TABLE_ALIGNMENT.LEFT
    hrow = tbl.add_row()
    for i, h in enumerate(headers):
        c = hrow.cells[i]
        c.width = col_widths[i]
        run = c.paragraphs[0].add_run(h)
        set_font(run, size=10, bold=True, color=RGBColor(0xFF,0xFF,0xFF))
        # shade header cell
        tcPr = c._tc.get_or_add_tcPr()
        shd = OxmlElement('w:shd')
        shd.set(qn('w:val'), 'clear')
        shd.set(qn('w:color'), 'auto')
        shd.set(qn('w:fill'), '165D76')
        tcPr.append(shd)
    for data_row in rows:
        drow = tbl.add_row()
        for i, cell_text in enumerate(data_row):
            c = drow.cells[i]
            c.width = col_widths[i]
            run = c.paragraphs[0].add_run(str(cell_text))
            set_font(run, size=10, color=C_BODY)
            c.paragraphs[0].paragraph_format.space_after = Pt(2)
    doc.add_paragraph()
    return tbl

def page_break():
    doc.add_page_break()

def spacer(pt=6):
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(pt)


# ═══════════════════════════════════════════════════════════════════════════════
# COVER PAGE
# ═══════════════════════════════════════════════════════════════════════════════

spacer(30)
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
p.paragraph_format.space_after = Pt(6)
r = p.add_run("MOTORCYCLE SIMULATOR")
set_font(r, size=28, bold=True, color=C_TITLE)

p2 = doc.add_paragraph()
p2.alignment = WD_ALIGN_PARAGRAPH.CENTER
p2.paragraph_format.space_after = Pt(4)
r2 = p2.add_run("Vehicle Dynamics Console")
set_font(r2, size=16, bold=False, color=C_H1)

p3 = doc.add_paragraph()
p3.alignment = WD_ALIGN_PARAGRAPH.CENTER
p3.paragraph_format.space_after = Pt(2)
r3 = p3.add_run("User Manual — Software Operation and Calculation Reference")
set_font(r3, size=12, italic=True, color=C_H3)

spacer(12)
p4 = doc.add_paragraph()
p4.alignment = WD_ALIGN_PARAGRAPH.CENTER
r4 = p4.add_run("Honda CBR600RR Reference Configuration  |  Pacejka MF Tire Model  |  PyChrono Multibody Physics")
set_font(r4, size=10, italic=True, color=C_H3)

spacer(60)
p5 = doc.add_paragraph()
p5.alignment = WD_ALIGN_PARAGRAPH.CENTER
r5 = p5.add_run("Version 3.0  (Milestone M3)   —   April 2026")
set_font(r5, size=10, color=C_H3)

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# TABLE OF CONTENTS (manual)
# ═══════════════════════════════════════════════════════════════════════════════

heading1("Table of Contents")
toc_items = [
    ("1", "Quick Start — Launching the Software"),
    ("2", "Application Overview — Three Tabs"),
    ("3", "Tab 1: Inputs and Parameters"),
    ("  3.1", "Motorcycle Geometry Group"),
    ("  3.2", "Mass and Brake Model Group"),
    ("  3.3", "Suspension Parameters"),
    ("  3.4", "Track and Disturbance Inputs"),
    ("  3.5", "Track Mode Selection"),
    ("  3.6", "Applying Parameters to the Simulation"),
    ("4", "Tab 2: Analysis and Mathematics"),
    ("  4.1", "Analytical Metrics Panel"),
    ("  4.2", "Dynamic Fz Weight Transfer Preview"),
    ("  4.3", "Stopping Distance Graph"),
    ("  4.4", "Stability Frequencies Bar Chart"),
    ("  4.5", "Interpretation Rules"),
    ("5", "Tab 3: Live Simulation and Graphs"),
    ("  5.1", "Connection Status Bar"),
    ("  5.2", "3D Motorcycle Viewport"),
    ("  5.3", "Rider Control Sliders"),
    ("  5.4", "Chart Panel — All Eight Graphs"),
    ("6", "All Calculations Explained"),
    ("  6.1", "Braking Calculations"),
    ("  6.2", "Suspension Ride Frequency"),
    ("  6.3", "Stability Modes (Wobble, Weave, Capsize)"),
    ("  6.4", "Pacejka Tire Model — Step by Step"),
    ("  6.5", "Combined Slip Scaling"),
    ("  6.6", "Aligning Torque"),
    ("  6.7", "Relaxation Dynamics (Slip Lag)"),
    ("  6.8", "Weight Transfer (Dynamic Fz)"),
    ("  6.9", "Suspension Spring and Damper"),
    ("7", "Default Parameter Values (CBR600RR)"),
    ("8", "Troubleshooting"),
    ("9", "Glossary of Terms"),
]
for num, title in toc_items:
    p = doc.add_paragraph()
    p.paragraph_format.space_after = Pt(3)
    r_num  = p.add_run(f"{num:<6}")
    r_title = p.add_run(title)
    set_font(r_num,   size=10, bold=True,  color=C_H1)
    set_font(r_title, size=10, bold=False, color=C_BODY)

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 1 — QUICK START
# ═══════════════════════════════════════════════════════════════════════════════

heading1("1   Quick Start — Launching the Software")

body("The simulator consists of two separate processes that must run at the same time: the Python physics backend and the Electron dashboard window. A desktop launcher handles both automatically.")

heading2("Starting the application")
numbered("Find the Motorcycle Dashboard icon on the Desktop.")
numbered("Double-click it.")
numbered("A dark Electron window labelled Motorcycle Simulator — Live Dashboard opens. The Python simulator starts in the background and binds to port 8765.")
numbered("Switch to the Live Simulation tab. The status bar at the top will show a green Connected dot within a few seconds.")
numbered("You are ready. Move the Throttle slider to begin.")

heading2("Closing the application")
body("Close the Electron window normally (click ×). The launcher script detects this and automatically kills the Python backend process, freeing port 8765. You do not need to kill anything manually.")

note_box("If you see 'Disconnected — retrying' for more than 10 seconds after opening, check that the backend log at /tmp/motorcycle_dashboard_logs/simulator.log does not contain an error. The most common cause is a missing Python dependency.")

heading2("Log files")
table_2col([
    ("launcher.log",  "Lifecycle events: backend start, stop, port detection. First place to check if the app fails to open."),
    ("simulator.log", "Full Python physics output: speed, roll, Fz readings every simulated second."),
    ("dashboard.log", "Electron / npm output. Usually empty unless there is a JavaScript error."),
], header=["File (in /tmp/motorcycle_dashboard_logs/)", "Contents"])

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 2 — APPLICATION OVERVIEW
# ═══════════════════════════════════════════════════════════════════════════════

heading1("2   Application Overview — Three Tabs")

body("The dashboard is organised into three tabs accessible from the tab bar at the top of the window. Each tab has a distinct purpose in the workflow.")

table_header_row([
    ["Inputs and Parameters",      "Set every physical property of the motorcycle before running. All changes here are staged — nothing is sent to the simulator until you click Apply."],
    ["Analysis and Mathematics",   "Instantly calculates stopping distances, stability frequencies, suspension ride frequencies, and weight transfer previews from the staged parameters. No simulation required — all results are analytical formulae."],
    ["Live Simulation and Graphs", "Real-time 3D view, rider control sliders, and eight live telemetry charts. All data comes from the running Python backend over WebSocket at 50 frames per second."],
], headers=["Tab", "Purpose"],
   col_widths=[Inches(2.0), Inches(4.2)])

body("Recommended workflow: set parameters → review analysis → apply → observe live simulation.")

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 3 — INPUTS AND PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════

heading1("3   Tab 1: Inputs and Parameters")

body("This tab contains every tunable property of the motorcycle. Parameters are grouped into three panels on the left and a track mode selector on the right. All fields accept direct numeric entry. Values are validated when applied.")

# 3.1
heading2("3.1   Motorcycle Geometry Group")

table_header_row([
    ["Wheelbase",              "m",     "1.37",  "Distance between front and rear axle centres measured along the ground. Longer wheelbase gives more stability but slower steering response. CBR600RR: 1.37 m."],
    ["Front Wheel Diameter",   "m",     "0.598", "Total outer diameter of front wheel and tyre (not radius). Used to compute wheel radius and contact patch height. CBR600RR 120/70-17 tyre: 0.598 m."],
    ["Rear Wheel Diameter",    "m",     "0.636", "Total outer diameter of rear wheel and tyre. CBR600RR 180/55-17 tyre: 0.636 m."],
    ["Front Tyre Width",       "m",     "0.12",  "Nominal section width of the front tyre. Used in the 3D visualisation only (tyre torus radius). Does not affect physics."],
    ["Rear Tyre Width",        "m",     "0.18",  "Nominal section width of the rear tyre. Used in the 3D visualisation only."],
    ["Rake Angle",             "deg",   "24.0",  "Angle between the steering axis and the vertical, measured in degrees. Higher rake (more laid-back fork) gives more stability but heavier steering. CBR600RR: 24°."],
    ["Trail",                  "m",     "0.097", "Distance between where the steering axis projects onto the ground and where the tyre contacts the ground. Trail provides self-aligning torque. CBR600RR: 97 mm."],
    ["Centre of Gravity Height","m",    "0.58",  "Height of the combined motorcycle-and-rider centre of gravity above the ground. Affects weight transfer magnitude and capsize speed. Typical range: 0.50–0.65 m."],
], headers=["Parameter", "Unit", "Default", "Description"],
   col_widths=[Inches(1.7), Inches(0.5), Inches(0.6), Inches(3.4)])

# 3.2
heading2("3.2   Mass and Brake Model Group")

table_header_row([
    ["Total Mass",             "kg",    "189",   "Combined mass of motorcycle and rider. CBR600RR dry weight 114 kg + 75 kg rider = 189 kg. Affects all inertial responses and static tyre loads."],
    ["Front Weight Fraction",  "ratio", "0.46",  "Fraction of total static weight carried by the front wheel. 0.46 means 46% front, 54% rear. Controls static Fz split. Typical range: 0.42–0.50."],
    ["Maximum Engine Torque",  "N·m",   "80",    "Peak drive torque at the rear wheel shaft. The throttle slider scales this: at 100% throttle, rear wheel torque = this value. CBR600RR: ~80 N·m at the wheel."],
    ["Maximum Brake Torque",   "N·m",   "250",   "Total combined brake torque capacity. Front brake applies 40% of this, rear brake applies 60%. Full front + rear braking = this total. Increase to stop shorter."],
], headers=["Parameter", "Unit", "Default", "Description"],
   col_widths=[Inches(1.7), Inches(0.5), Inches(0.6), Inches(3.4)])

# 3.3
heading2("3.3   Suspension Parameters")

body("These values control the spring-damper pair at each end of the motorcycle. They directly affect suspension travel in the 3D view and the Suspension Travel chart.")

table_header_row([
    ["Front Spring Rate",       "N/m",   "18500", "Stiffness of the front fork spring. Higher = stiffer fork, less travel under load. Typical sport range: 15 000–22 000 N/m."],
    ["Rear Spring Rate",        "N/m",   "42000", "Stiffness of the rear shock spring. Rear is always stiffer because it carries more static load. Typical sport range: 35 000–55 000 N/m."],
    ["Front Damping Coefficient","N·s/m","900",   "Viscous damping of the front fork. Controls how quickly oscillations die after a bump. Target damping ratio: 0.2–0.4 (see Analysis tab)."],
    ["Rear Damping Coefficient", "N·s/m","1800",  "Viscous damping of the rear shock. Same principle as front. The Analysis tab shows the resulting damping ratio for both ends."],
], headers=["Parameter", "Unit", "Default", "Description"],
   col_widths=[Inches(1.9), Inches(0.7), Inches(0.6), Inches(3.0)])

note_box("The static spring compression at rest is computed automatically from the spring rate and static load. You do not need to set a preload — the simulator calculates the rest length such that the spring is already supporting the static weight when the simulation starts.")

# 3.4
heading2("3.4   Track and Disturbance Inputs")

body("These fields configure the road environment and reference speed for analytical calculations. They also affect which scenario is used when a non-flat track mode is selected.")

table_header_row([
    ["Track Length",        "m",    "400",   "Total road segment length used for scenario planning. Affects distance displayed in track-mode animations (not currently live in the physics backend)."],
    ["Corner Radius",       "m",    "80",    "Reference corner radius for cornering analysis. Used in the bump acceleration index in the Analysis tab."],
    ["Bump Height",         "m",    "0.015", "Vertical height of bump events (15 mm default). Used in the disturbance index calculation in the Analysis tab."],
    ["Pothole Depth",       "m",    "0.030", "Depth of pothole events (30 mm default). Used in the pothole severity index."],
    ["Friction Coefficient","—",    "0.9",   "Tyre-road friction coefficient μ. Dry asphalt: 0.85–1.0. Wet: 0.5–0.7. Affects stopping distance and the friction circle limit shown in the live charts."],
    ["Reference Speed",     "m/s",  "12",    "Input speed for all analytical calculations (stopping distance, stability frequencies). Does not change the simulation reset speed — that is set by v₀ in the Control Panel."],
], headers=["Parameter", "Unit", "Default", "Description"],
   col_widths=[Inches(1.7), Inches(0.5), Inches(0.6), Inches(3.4)])

# 3.5
heading2("3.5   Track Mode Selection")
body("Four track mode buttons appear on the right side panel. The selected mode is sent to the simulator when you click Apply. In the current release the physics backend always runs on a flat infinite plane regardless of the mode — the selection is stored for future road modules.")

table_2col([
    ("Flat Road",         "Infinite flat asphalt at z = 0. Default. No road disturbances."),
    ("Bumps",             "Sinusoidal road surface with Bump Height amplitude. Future module."),
    ("Potholes",          "Discrete step disturbances of Pothole Depth. Future module."),
    ("Mixed Disturbance", "Combination of bumps and potholes. Future module."),
], header=["Mode", "Description"])

# 3.6
heading2("3.6   Applying Parameters to the Simulation")
body("All changes in this tab are staged locally. Nothing is sent to the running physics backend until you explicitly click Apply Inputs To Simulation.")
body("When you click Apply:")
numbered("All current parameter values are packaged into a single JSON message and sent over WebSocket to the Python backend.")
numbered("A reset command is also sent — the simulation rebuilds the motorcycle model with the new geometry, mass, and spring rates.")
numbered("The simulation resets to the v₀ speed set in the Live Simulation control panel.")
numbered("A green timestamp badge confirms the values were received.")
note_box("Applying mid-run will briefly interrupt the telemetry stream while the backend rebuilds the model. This is normal — the connection restores within 1–2 seconds.", prefix="NOTE")

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 4 — ANALYSIS TAB
# ═══════════════════════════════════════════════════════════════════════════════

heading1("4   Tab 2: Analysis and Mathematics")

body("This tab shows computed results using only the staged parameter values. No simulation is needed. The calculations update instantly whenever you change a value in the Parameters tab.")

# 4.1
heading2("4.1   Analytical Metrics Panel")

body("Eight metric cards show key performance figures:")

table_2col([
    ("Stopping Distance [m]",       "Distance to stop from Reference Speed under the effective deceleration. Formula: d = v² / (2·a_eff). Rises quadratically with speed."),
    ("Stopping Time [s]",           "Time to reach standstill. Formula: t = v / a_eff."),
    ("Effective Deceleration [m/s²]","The smaller of friction-limited deceleration (μ·g) and brake-torque-limited deceleration. Both limits are evaluated and the binding one is used."),
    ("Wobble Frequency [Hz]",       "Natural frequency of the fork steering oscillation. Speed-dependent. See Section 6.3 for the formula."),
    ("Weave Frequency [Hz]",        "Natural frequency of the coupled yaw-roll oscillation. Rises linearly with speed. See Section 6.3."),
    ("Capsize Speed [m/s]",         "Speed above which the lean-fall instability becomes self-stabilising. At speeds above this value the motorcycle does not need active balance correction to resist capsize. See Section 6.3."),
    ("Front Damping Ratio [—]",     "Dimensionless measure of front suspension damping. 1.0 = critically damped, < 1 = underdamped (bouncy), > 1 = overdamped (sluggish). Good target: 0.2–0.4."),
    ("Rear Damping Ratio [—]",      "Same as front but for the rear shock."),
], header=["Metric", "Meaning"])

# 4.2
heading2("4.2   Dynamic Fz — Weight Transfer Preview")

body("This chart shows how the normal load on each wheel changes as the motorcycle decelerates. The X-axis is deceleration in units of g (gravitational acceleration). The Y-axis is wheel load in Newtons.")
body("Lime green line = front wheel load (Fz_f). Cyan dashed line = rear wheel load (Fz_r).")
body("Under braking the front load rises and the rear load falls. At maximum braking the front load can reach 70–80% of total vehicle weight. Below the chart, exact static and maximum braking loads are printed.")
body("The calculation uses: ΔFz = m · a · h_cg / L, where m is total mass, a is deceleration, h_cg is CG height, and L is wheelbase. This is the longitudinal weight transfer formula.")

# 4.3
heading2("4.3   Stopping Distance Graph")

body("The red curve shows stopping distance versus speed from zero up to 1.8× the Reference Speed. The relationship is always parabolic (d ∝ v²) meaning doubling speed quadruples stopping distance. This is the most important braking insight in the analysis tab.")

# 4.4
heading2("4.4   Stability Frequencies Bar Chart")

body("Four horizontal bars on the right panel show:")
table_2col([
    ("Wobble",      "Steering oscillation. Target: well above 6 Hz so it is too fast to feel."),
    ("Weave",       "Yaw-roll coupled mode. Target: below 4 Hz in normal riding to stay controllable."),
    ("Front Ride",  "Vertical bounce of front suspension. Target: 1–2 Hz for comfort, up to 4 Hz for sport."),
    ("Rear Ride",   "Vertical bounce of rear suspension. Usually slightly higher than front."),
])
body("The bar width shows the frequency relative to the scale maximum for each mode. A longer bar does not necessarily mean better — the interpretation panel explains the correct ranges.")

# 4.5
heading2("4.5   Interpretation Rules")
body("The side panel lists these guidelines:")
bullet("Capsize speed below the current reference speed means the bike is dynamically self-stabilising in the capsize mode at that speed.")
bullet("Wobble above 6 Hz and weave below 4 Hz is a well-separated controllable pairing. If they get close, the modes can couple and become harder to damp.")
bullet("Damping ratio 0.2–0.4 is the standard target for motorcycle suspension. Below 0.2 the suspension bounces excessively. Above 0.4 the suspension is too stiff to absorb road inputs.")
bullet("At maximum braking the front Fz should stay below approximately 80% of total load to avoid losing rear tyre grip entirely, which causes instability.")

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 5 — LIVE SIMULATION
# ═══════════════════════════════════════════════════════════════════════════════

heading1("5   Tab 3: Live Simulation and Graphs")

body("This tab shows real-time data from the running Python physics backend. All charts update at approximately 50 frames per second. Up to 1000 data points are kept in the rolling buffer.")

# 5.1
heading2("5.1   Connection Status Bar")

body("The status bar runs across the top of this tab. It contains:")
bullet("Green dot + Connected: WebSocket link to the backend is active.")
bullet("Red dot + Disconnected — retrying: The backend is not reachable. Auto-retry every 2 seconds.")
bullet("Live readouts: simulation time t, longitudinal speed vx, lean angle, yaw rate, front and rear normal loads Fz, steer angle, and steer torque. All values update at 100 ms intervals.")
bullet("Pause / Resume button: freezes the simulation physics without closing the connection. Useful to inspect a specific moment.")
bullet("Clear button: empties the 1000-point chart buffer. The charts restart from zero.")
bullet("Export CSV button: downloads all buffered telemetry as a comma-separated file. Useful for post-session analysis in Excel or Python.")

# 5.2
heading2("5.2   3D Motorcycle Viewport")

body("The large panel on the left renders a live Three.js 3D model of the motorcycle. The visualisation updates in real time as the simulation runs.")
table_2col([
    ("Frame lean",    "The main body leans to match the simulated roll angle. Red at lean angles beyond ±5°."),
    ("Fork steer",    "The front fork assembly rotates about the steering axis to show steer angle."),
    ("Wheel spin",    "Both wheels rotate at the correct angular velocity for the current speed."),
    ("Fork compression","The fork legs slide along the fork axis to show suspension travel."),
    ("Swingarm",      "The rear swingarm pivots to show rear suspension travel."),
    ("Mouse orbit",   "Left-click drag to orbit. Scroll to zoom. The model is always visible regardless of orientation."),
])

body("The overlay readout in the bottom-left corner of the viewport shows the seven most important real-time values without needing to read the charts.")

# 5.3
heading2("5.3   Rider Control Sliders")

body("Six controls sit below the 3D viewport. They send commands to the Python backend over WebSocket in real time — there is no delay or buffering.")

table_header_row([
    ["Throttle",     "0–100%", "Rear wheel drive torque as a fraction of Maximum Engine Torque. At 100% the rear wheel receives the full engine torque value set in parameters."],
    ["Brake F",      "0–100%", "Front brake as a fraction of brake capacity. The front brake applies 40% of Maximum Brake Torque. Example: at 100% Front Brake = 0.40 × 250 = 100 N·m at the front wheel."],
    ["Brake R",      "0–100%", "Rear brake as a fraction of brake capacity. The rear brake applies 60% of Maximum Brake Torque. Example: at 100% Rear Brake = 0.60 × 250 = 150 N·m."],
    ["Steer [N·m]",  "−30 to +30 N·m", "Torque applied about the steering axis. Positive = steer right (which causes the bike to turn LEFT due to countersteering). The slider auto-centres to zero when you release the mouse."],
    ["v₀ [m/s]",     "1–15 m/s", "Initial speed used when you press Reset. Does not change speed while the simulation is running."],
    ["Pause / Reset", "—",  "Pause freezes physics. Reset zeroes all inputs, rebuilds the model with current parameters, and restarts at v₀."],
], headers=["Control", "Range", "What It Does"],
   col_widths=[Inches(1.3), Inches(1.2), Inches(3.7)])

note_box("Countersteering: pushing the handlebar to the RIGHT makes the motorcycle turn LEFT. This is physically correct and is the mechanism used by all riders above ~15 km/h. The simulation demonstrates this if you apply a brief positive steer torque pulse and observe the yaw rate in the status bar.", prefix="PHYSICS NOTE")

# 5.4
heading2("5.4   Chart Panel — All Eight Graphs")

body("Eight chart cells are arranged in the right half of the Live Simulation tab. All charts share the same horizontal time axis (seconds). The buffer holds 1000 data points at 50 Hz = 20 seconds of history.")

table_header_row([
    ["Normal Loads",       "Top row, full width", "Front (lime) and rear (cyan) normal loads Fz in Newtons vs time. The coloured bar below shows the current front-to-rear load fraction. Red bar = front overloaded (>65%). Orange = rear very light (<30%)."],
    ["Longitudinal & Lateral Forces", "Row 2 left", "Fx (longitudinal: traction/braking) and Fy (lateral: cornering) for both tyres. Four traces. Under acceleration: positive Fx_rear. Under braking: negative Fx."],
    ["Friction Circle",    "Row 2 right", "Scatter plot of normalised front tyre usage: Fx/Fz on X-axis, Fy/Fz on Y-axis. The pink circle is the friction limit at μ = 0.9. Points inside the circle = grip available. Points on or outside = tyre at limit."],
    ["Slip",               "Row 3 left", "Slip angle α (degrees, left axis, purple) and slip ratio κ (dimensionless, right axis, orange) for both tyres. Solid = front, dashed = rear."],
    ["Aligning Torque",    "Row 3 right", "Self-aligning torque Mz (N·m) for both tyres vs time. Green = front, purple dashed = rear. Positive Mz tends to steer the wheel back to zero slip. The shaded area shows direction of Mz."],
    ["Chassis Orientation","Row 4 left", "Roll (red), pitch (amber dashed), and yaw (blue dotted) angles in degrees on the left axis. Roll rate and yaw rate in degrees/second on the right axis."],
    ["Suspension Travel",  "Row 4 right","Fork compression (amber) and rear shock compression (purple dashed) in millimetres vs time. Positive values mean compressed from the natural length. Static values at cruise: ~45 mm front, ~43 mm rear."],
], headers=["Chart", "Position", "What to Read"],
   col_widths=[Inches(1.6), Inches(1.2), Inches(3.4)])

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 6 — ALL CALCULATIONS EXPLAINED
# ═══════════════════════════════════════════════════════════════════════════════

heading1("6   All Calculations Explained")

body("This section explains every formula used in the software, written for a mechanical engineer. All physics is in SI units.")

# 6.1
heading2("6.1   Braking Calculations")

heading3("Friction-limited deceleration")
body("A tyre can generate a maximum horizontal force equal to μ × Fz, where μ is the friction coefficient and Fz is the normal load. The maximum deceleration limited by tyre grip is:")
body("       a_friction = μ × g")
body("For μ = 0.9: a_friction = 0.9 × 9.81 = 8.83 m/s²")

heading3("Brake-torque-limited deceleration")
body("The brakes generate a torque at the wheel. That torque divided by the wheel radius gives the braking force. The deceleration is:")
body("       a_brake = Brake_Torque_Max / (m × R_avg)")
body("where R_avg is the average of front and rear wheel radii. For default parameters: a_brake = 250 / (189 × 0.308) = 4.30 m/s². The brakes are the binding constraint in this case.")

heading3("Effective deceleration")
body("The smaller of the two limits is used:")
body("       a_eff = min(a_friction, a_brake)  but at least 0.5 m/s²")

heading3("Stopping distance and time")
body("From constant deceleration kinematics:")
body("       d = v² / (2 × a_eff)")
body("       t = v / a_eff")
body("These are the values shown in the Analytical Metrics cards.")

# 6.2
heading2("6.2   Suspension Ride Frequency")

body("Each end of the suspension behaves like a spring-mass system. The undamped natural frequency (ride frequency) is:")
body("       f_ride = (1 / 2π) × √(k / m_supported)")
body("where k is the spring rate and m_supported is the mass carried by that spring:")
body("       m_front_supported = Total_Mass × Front_Weight_Fraction")
body("       m_rear_supported  = Total_Mass × (1 − Front_Weight_Fraction)")
body("For default parameters:")
body("       f_front = (1/2π) × √(18500 / (189 × 0.46)) = (1/2π) × √213 = 2.32 Hz")
body("       f_rear  = (1/2π) × √(42000 / (189 × 0.54)) = (1/2π) × √412 = 3.23 Hz")

heading3("Damping ratio")
body("The damping ratio ζ shows how quickly oscillations die out:")
body("       ζ = c / (2 × √(k × m_supported))")
body("ζ = 1.0 means critically damped (no oscillation). ζ < 1 means the suspension bounces. ζ > 1 means overdamped (stiff, slow response). The target for motorcycle suspension is ζ = 0.2–0.4.")

# 6.3
heading2("6.3   Stability Modes: Wobble, Weave, and Capsize")

heading3("The three classical modes (Sharp 1971)")
body("Every two-wheeled vehicle has three characteristic motions when disturbed. These were first computed analytically by Robin Sharp in 1971 and remain the foundation of motorcycle dynamics theory.")

table_2col([
    ("Wobble",   "High-frequency oscillation of the steering assembly. The fork and front wheel shake left-right at typically 7–12 Hz. Caused by tyre cornering stiffness reacting against geometric trail self-alignment. It is a complex eigenvalue (oscillatory). Normally well-damped at road speeds."),
    ("Weave",    "Medium-frequency coupled oscillation of yaw, roll, and steering at typically 1–4 Hz. The whole motorcycle sways. Frequency increases with speed. Stable at most road speeds for normal motorcycles. The simulation validates this in M2 gate tests."),
    ("Capsize",  "Slow falling-over tendency. At low speed it is unstable (you need to actively balance). At higher speeds the geometry provides enough self-aligning force that capsize becomes stable. The speed at which it switches is the capsize speed."),
])

heading3("Wobble frequency formula")
body("The analysis tab uses the linearised formula from Sharp (1971):")
body("       ω²_wobble = (C_αf × t + m_front × v² × t / L) / I_steer")
body("       f_wobble  = ω_wobble / (2π)")
body("where C_αf is front tyre cornering stiffness (≈ 10 × Fz_front, N/rad), t is trail, v is speed, L is wheelbase, m_front is front supported mass, and I_steer = 0.5 kg·m² (estimated fork + wheel steering inertia). The formula shows wobble frequency rises with speed.")

heading3("Weave frequency formula")
body("The approximate speed-dependent weave frequency:")
body("       f_weave = v × √(12 × h_cg) / (2π × L²)")
body("where h_cg is CG height and L is wheelbase. This rises linearly with speed, consistent with Sharp (1971).")

heading3("Capsize speed formula")
body("From Sharp's linearised analysis, the capsize eigenvalue transitions from unstable to stable at:")
body("       v_capsize = √(g × t × L / (h_cg × f_front))")
body("where f_front is the front weight fraction. For default parameters:")
body("       v_capsize = √(9.81 × 0.097 × 1.37 / (0.58 × 0.46)) = √(3.44) = 1.86 m/s")
body("This is very low (≈ 6.7 km/h), meaning the CBR600RR is self-stabilising in capsize at any normal riding speed.")

# 6.4
heading2("6.4   Pacejka Tire Model — Step by Step")

body("The Pacejka Magic Formula is the international standard for tyre force modelling. It was developed by Hans Pacejka at TU Delft and is used in Formula 1, road cars, and motorcycles. The simulation uses it for both front and rear tyres.")

heading3("Step 1 — Peak force capacity D")
body("       D = μ × Fz")
body("μ is the friction coefficient, Fz is the normal load. This is the maximum force the tyre can generate in any direction. At Fz = 853 N (front static) and μ = 0.9: D = 767 N.")

heading3("Step 2 — Stiffness factor B")
body("       B = (k_alpha × Fz) / (C × D + ε)")
body("k_alpha = 12.0 is the cornering stiffness coefficient (a tyre property). C = 1.80 is the shape factor. The product B × C × D is the cornering stiffness dFy/dα at α = 0, which equals approximately 8800 N/rad for a medium-compound sport tyre.")

heading3("Step 3 — Shape function φ")
body("       φ = B × α'  −  E × (B × α' − arctan(B × α'))")
body("E is the curvature factor (set to 0 in this model, so φ = B × α'). α' is the relaxed slip angle. E > 0 would bend the peak of the curve.")

heading3("Step 4 — Pacejka lateral force Fy_pac")
body("       Fy_pac = D × sin(C × arctan(φ))")
body("The arctan maps all slip angles to the range (−π/2, +π/2). sin(C × arctan(·)) creates the classic S-shaped Pacejka curve: rises steeply, peaks at 4–8°, then plateaus.")

heading3("Step 5 — Add camber thrust")
body("       Fy = clamp(Fy_pac + k_gamma × Fz × γ,  −D,  +D)")
body("k_gamma = 0.90. When the motorcycle leans (camber angle γ ≠ 0), the tyre generates lateral force even at zero slip angle. This is camber thrust. The clamp ensures total lateral force never exceeds the friction limit D.")

heading3("Step 6 — Longitudinal force Fx₀")
body("       Fx₀ = μ × Fz × tanh(Kx × κ)")
body("Kx = 12.0. κ (kappa) is the longitudinal slip ratio. Positive κ = traction (wheel spinning faster than road). Negative κ = braking. The tanh function gives linear response at small κ and saturates at the friction limit μ × Fz.")

# 6.5
heading2("6.5   Combined Slip Scaling")

body("A tyre generating both lateral and longitudinal force simultaneously cannot produce full force in both directions — the contact patch rubber has a finite friction budget. If the tyre uses 80% of grip for braking, only 20% remains for cornering.")
body("The Pacejka (2012) solution uses smooth coupling functions G:")

body("       Gxα = cos(Cxα × arctan(Bxα × α'))        [reduces Fx due to lateral slip]")
body("       Gyκ = cos(Cyκ × arctan(Byκ × κ))          [reduces Fy due to longitudinal slip]")
body("       Fx  = Gxα × Fx₀       Fy = Gyκ × Fy₀")

body("Both G factors are always between 0 and 1 (since Cxα = Cyκ = 1.0, the cosine argument stays in −π/2 to +π/2). G = 1 when the other slip is zero. G → 0 when the other slip is very large. The functions are infinitely smooth with no branching, which is important for numerical stability.")

# 6.6
heading2("6.6   Aligning Torque Mz")

heading3("Pneumatic trail tp")
body("The lateral force in the contact patch does not act at the tyre centreline. It acts slightly behind centre at the pneumatic trail distance tp:")
body("       tp = tp₀ × cos(Ct × arctan(Bt × α'))")
body("tp₀ = 30 mm is the trail at zero slip. As slip angle increases, tp decreases. At a critical angle called the trail reversal angle (≈8.3° for default settings), tp reaches zero and the self-aligning effect disappears. Beyond this, tp becomes negative, which is a known phenomenon in high-slip conditions.")

heading3("Aligning torque calculation")
body("       Mz = −Fy × tp  +  K_gamma_Mz × γ × Fz")
body("The first term is the main self-aligning torque. It tends to steer the wheel back toward zero slip (negative sign because Fy > 0 with positive slip creates a restoring torque about the steering axis). The second term is a small camber correction. Crucially, this uses the COMBINED Fy (after Gyκ scaling), not the pure-slip value, so the aligning torque is always physically consistent with the actual lateral force generated.")

# 6.7
heading2("6.7   Relaxation Dynamics — Why Tyres Respond With a Lag")

heading3("The physical reason")
body("When the steering angle changes, the tyre does not instantly generate its new lateral force. The rubber in the contact patch must physically deform before it can produce a lateral force in the new direction. This takes a finite travel distance — approximately equal to the contact patch half-length, called the relaxation length σ ≈ 0.30 m. At 10 m/s, one relaxation length passes in 30 ms.")

heading3("The relaxation ODE")
body("The relaxation is modelled as a first-order differential equation:")
body("       (σ / Vx) × dα'/dt + α' = α")
body("where α is the instantaneous kinematic slip angle, α' is the filtered (relaxed) slip angle actually used for force calculation, and σ/Vx is the time constant τ. At Vx = 10 m/s, τ = 0.03 s.")

heading3("Zero-Order Hold (ZOH) integration")
body("The equation is solved exactly at each timestep using the ZOH method (A-stable, no step-size limit):")
body("       α'[k+1] = α + (α'[k] − α) × exp(−|Vx| × dt / σ)")
body("This is the formula implemented in the code. It is mathematically exact for a first-order linear ODE with constant input over each timestep, unlike Euler integration which can go unstable at large timesteps.")

heading3("The same method is applied to longitudinal slip κ")
body("       κ'[k+1] = κ_kin + (κ'[k] − κ_kin) × exp(−Vx × dt / σ_kappa)")
body("This prevents numerical slip noise (from the PyChrono contact model) from accumulating into a growing braking force during free rolling.")

# 6.8
heading2("6.8   Weight Transfer — Dynamic Fz")

body("When the motorcycle accelerates or brakes, weight shifts between the front and rear wheels. This affects tyre grip and is critical for understanding braking performance.")

heading3("Formula")
body("       ΔFz_long = m × ax × h_cg / L")
body("Under braking (ax > 0): front wheel load increases, rear decreases.")
body("Under acceleration (ax < 0): rear wheel load increases, front decreases.")
body("The simulation uses a 0.1-second exponential moving average to filter the acceleration signal before computing ΔFz. This prevents high-frequency numerical noise from the integrator from appearing as large instantaneous load spikes.")

heading3("Lateral weight transfer (M3)")
body("During cornering at lean angle φ, the centripetal acceleration has a component that also shifts load front-to-rear:")
body("       ΔFz_lat = m × ay × h_cg × sin(roll) / L")
body("Both terms are combined: ΔFz = ΔFz_long + ΔFz_lat. Total weight is conserved: Fz_front + Fz_rear = m × g at all times.")

# 6.9
heading2("6.9   Suspension Spring and Damper — TSDA Model")

heading3("Front fork (telescopic)")
body("The front fork is modelled as two rigid bodies (upper and lower tubes) connected by a prismatic joint (slides along the fork axis) and a TSDA (Translational Spring-Damper-Actuator) link. The TSDA force along the fork axis is:")
body("       F_spring = k_f × (rest_length − current_length)")
body("       F_damper = c_f × (d/dt)(current_length)")
body("       F_total  = F_spring + F_damper")

heading3("Rest length pre-load")
body("The rest length is computed so the spring is already compressed by the correct amount to support the static front load at the design ride height. This means when the simulation starts, the fork is already at the correct position with no initial bouncing.")
body("       δ_front = F_static_front / (k_front × cos(rake))")
body("       rest_length_front = design_length + δ_front")
body("The cos(rake) factor converts the vertical load component to a force along the fork axis (inclined at the rake angle).")

heading3("Suspension travel readout")
body("The charts and 3D view show suspension travel as compression relative to the rest length:")
body("       fork_travel = rest_length − current_length   (positive = compressed)")
body("At cruise the fork is compressed by approximately 45 mm (the static preload). Under hard braking, it compresses further as weight transfers to the front. During acceleration it may slightly extend.")

heading3("Rear suspension")
body("The rear shock connects the main frame to the swingarm at a specific moment arm. The spring force creates a clockwise torque about the swingarm pivot that balances the counterclockwise torque from the rear tyre normal load. The preload is computed from this torque balance.")

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 7 — DEFAULT PARAMETERS
# ═══════════════════════════════════════════════════════════════════════════════

heading1("7   Default Parameter Values (Honda CBR600RR 2007)")

body("All default values represent the Honda CBR600RR with a 75 kg rider, based on Cossalter 'Motorcycle Dynamics' 2nd edition and Sharp (1994) benchmark parameters.")

table_header_row([
    ["Wheelbase",             "1.370 m",     "Front to rear axle"],
    ["Rake angle",            "24.0°",       "0.4189 rad — steering head angle from vertical"],
    ["Trail",                 "97 mm",       "0.097 m — mechanical trail at contact patch"],
    ["Front wheel diameter",  "598 mm",      "120/70-17 tyre — radius = 0.299 m"],
    ["Rear wheel diameter",   "636 mm",      "180/55-17 tyre — radius = 0.318 m"],
    ["Total mass",            "189 kg",      "114 kg dry bike + 75 kg rider"],
    ["CG height",             "0.580 m",     "Combined motorcycle + rider"],
    ["Front weight fraction", "0.46",        "46% front, 54% rear static split"],
    ["Max engine torque",     "80 N·m",      "At rear wheel shaft"],
    ["Max brake torque",      "250 N·m",     "Combined — 40% front, 60% rear"],
    ["Front spring rate",     "18 500 N/m",  "Telescopic fork spring"],
    ["Rear spring rate",      "42 000 N/m",  "Rear shock spring"],
    ["Front damping",         "900 N·s/m",   "Damping ratio ≈ 0.27 at static load"],
    ["Rear damping",          "1 800 N·s/m", "Damping ratio ≈ 0.27 at static load"],
    ["Friction coefficient",  "0.90",        "Dry asphalt, medium compound tyre"],
    ["Tyre relaxation length","0.30 m",      "Used for both lateral and longitudinal slip lag"],
    ["Frame roll inertia Ixx","9.5 kg·m²",   "About longitudinal axis through CG"],
    ["Frame pitch inertia Iyy","58.0 kg·m²", "About lateral axis through CG"],
    ["Frame yaw inertia Izz", "50.0 kg·m²",  "About vertical axis through CG"],
], headers=["Parameter", "Value", "Notes"],
   col_widths=[Inches(2.0), Inches(1.2), Inches(3.0)])

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 8 — TROUBLESHOOTING
# ═══════════════════════════════════════════════════════════════════════════════

heading1("8   Troubleshooting")

table_header_row([
    ["Dashboard shows Disconnected permanently",
     "The Python backend failed to start. Open /tmp/motorcycle_dashboard_logs/simulator.log in a text editor. Look for an error on the last line. Common causes: missing Python package (run pip install pychrono torch numpy scipy pyyaml websockets), wrong Python path, or the port was still in use from a crashed session. Close and re-open the app — the launcher kills any stale process on port 8765 at startup."],
    ["3D view is completely black",
     "Electron cannot create a WebGL context. The launcher uses SwiftShader software rendering which should work on any GPU. If the window is black, try running from a terminal: cd /home/dikshant/Desktop/Project vi grade/Startup/moto_sim/dashboard && npm start"],
    ["Charts show data but values look wrong (very large or NaN)",
     "Physics has gone unstable. Click Reset in the control panel. If it keeps happening, the spring rate or mass parameters may be physically unrealistic. Try restoring defaults by clicking Apply without changing any values."],
    ["Suspension travel reads 0 mm constantly",
     "This is normal at the very first frame before PyChrono settles. After 0.5 s of simulation it should read approximately 43–47 mm. If it stays at zero, the TSDA joint may have failed to initialise — check the simulator log."],
    ["Throttle slider has no effect on speed",
     "The motorcycle may already be at the speed where drag equals drive force, or brakes are partially applied. Confirm Brake F and Brake R sliders are at 0. Also check that Maximum Engine Torque is set above zero in parameters."],
    ["Countersteering seems backwards",
     "This is expected physics. Moving the handlebar right at speed causes the motorcycle to turn left. Apply a brief +2 N·m steer pulse and observe the yaw_rate readout go negative (turning left in SAE convention). This was validated in the M1 G6 test."],
    ["Export CSV is empty",
     "You need to be on the Live Simulation tab and have received at least one data frame. If the status bar shows Connected and frames are counting up but the export is empty, click Clear first then wait a few seconds before exporting again."],
], headers=["Symptom", "Fix"],
   col_widths=[Inches(2.0), Inches(4.2)])

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# SECTION 9 — GLOSSARY
# ═══════════════════════════════════════════════════════════════════════════════

heading1("9   Glossary of Terms")

terms = [
    ("Slip angle α (alpha)",
     "The angle between the direction the tyre is pointing and the direction it is actually moving. At zero α there is no cornering force. Peak cornering force occurs at approximately 4–8° for a motorcycle tyre. Measured in radians in the simulation, displayed in degrees in the charts."),
    ("Slip ratio κ (kappa)",
     "Dimensionless measure of longitudinal slip. κ = (R·ω − Vx) / Vx. Positive = traction (wheel spinning faster than road). Negative = braking (wheel spinning slower). Zero = free rolling."),
    ("Normal load Fz",
     "The vertical force pressing the tyre onto the road. Equals approximately half the vehicle weight at static, but changes dynamically under braking and acceleration (weight transfer). Determines the peak lateral and longitudinal force the tyre can generate."),
    ("Camber angle γ (gamma)",
     "The angle the tyre makes with the vertical when viewed from behind. When the motorcycle leans in a corner, the camber angle equals the lean angle. Camber generates lateral force even at zero slip angle (camber thrust)."),
    ("Friction coefficient μ",
     "The ratio of maximum horizontal force to normal load that a tyre can generate. Dry asphalt with a sport tyre: μ ≈ 0.85–1.0. Wet asphalt: μ ≈ 0.5–0.7. μ = 0.9 is the default."),
    ("Relaxed slip angle α'",
     "The filtered slip angle after the first-order relaxation ODE. This is what the Pacejka formula receives, not the instantaneous kinematic slip angle. The difference between α and α' represents the tyre memory effect."),
    ("Relaxation length σ",
     "The travel distance over which the tyre transitions from old lateral force to new lateral force after a steering change. Approximately equal to the contact patch half-length: 0.30 m for a motorcycle tyre."),
    ("Cornering stiffness",
     "dFy/dα at α = 0, the initial slope of the lateral force vs slip angle curve. Equals B × C × D in Pacejka notation. Approximately 8800 N/rad for the medium compound tyre in this simulation."),
    ("Pneumatic trail tp",
     "The distance behind the tyre centreline where the resultant lateral force effectively acts. Creates self-aligning torque. Decreases as slip angle increases. Reverses sign at the trail reversal angle (~8.3°)."),
    ("Aligning torque Mz",
     "The torque about the vertical axis at the contact patch. Mz = −Fy × tp + camber term. Tends to steer the wheel back to zero slip (self-alignment). Shown in the Aligning Torque chart."),
    ("Weave mode",
     "A coupled yaw-roll-steer oscillation at 1–4 Hz. The whole motorcycle sways side to side. Frequency increases with speed. Present in the simulation and validated in M2 gate test G5 (oscillatory sign reversals)."),
    ("Wobble mode",
     "A high-frequency steering oscillation at 7–12 Hz. Only the front fork and wheel oscillate. Caused by the interaction between tyre cornering stiffness and geometric trail. Normally well-damped."),
    ("Capsize mode",
     "The slow lean-fall instability. Unstable at low speed (you need to actively balance). Becomes self-stabilising above the capsize speed (typically ~1.9 m/s for the CBR600RR defaults)."),
    ("TSDA (Translational Spring-Damper-Actuator)",
     "The PyChrono joint type used for both front fork and rear shock. Acts as a spring and damper in series along a straight line between two attachment points."),
    ("ZOH (Zero-Order Hold)",
     "A numerical integration method for first-order linear ODEs that is unconditionally stable. Used for both the lateral and longitudinal slip relaxation in the tyre model."),
    ("HHT (Hilber-Hughes-Taylor) integrator",
     "The implicit time-stepping method used by PyChrono for multibody dynamics. Better stability than Runge-Kutta for stiff mechanical systems with constraints and contact."),
    ("Friction circle / ellipse",
     "The boundary of combined lateral and longitudinal tyre force. Points within the circle represent available grip. Points on the boundary mean the tyre is at the friction limit. Shown in the Friction Circle chart using normalised forces Fx/Fz and Fy/Fz."),
    ("SAE J670 coordinate frame",
     "The coordinate system used in this simulation: X forward, Y left, Z up. Right-hand convention. A positive yaw rate means turning left (nose moving toward +Y). A positive roll is lean to the right."),
    ("WebSocket",
     "The real-time communication channel between the Python physics backend and the Electron dashboard. Operates at 50 Hz (one frame every 20 ms). Port 8765 on localhost."),
]

for term, definition in terms:
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(5)
    p.paragraph_format.space_after  = Pt(2)
    run_t = p.add_run(f"{term}   ")
    run_d = p.add_run(definition)
    set_font(run_t, size=10.5, bold=True,  color=C_H1)
    set_font(run_d, size=10.5, bold=False, color=C_BODY)

page_break()


# ═══════════════════════════════════════════════════════════════════════════════
# FINAL PAGE — QUICK REFERENCE CARD
# ═══════════════════════════════════════════════════════════════════════════════

heading1("Quick Reference — Key Numbers and Limits")

table_header_row([
    ["Simulation rate",        "1 000 Hz",    "Physics timestep = 1 ms"],
    ["Dashboard update rate",  "50 Hz",       "One telemetry frame every 20 ms"],
    ["Chart buffer",           "1 000 points","= 20 seconds of history at 50 Hz"],
    ["WebSocket port",         "8765",        "localhost only"],
    ["Max steer torque",       "±30 N·m",     "Slider range in control panel (backend accepts ±50)"],
    ["Straight-line stability","±5° roll",    "M1 gate limit — should stay under this at cruise"],
    ["Weave gate",             "≥1 sign reversal in 2 s","M2 gate — confirms oscillatory mode"],
    ["Fork travel at cruise",  "~45 mm",      "Static preload from front weight"],
    ["Rear travel at cruise",  "~43 mm",      "Static preload from rear weight"],
    ["Peak slip angle",        "~4–8°",       "Where Pacejka Fy peaks for medium compound"],
    ["Trail reversal angle",   "~8.3°",       "Where Mz changes sign"],
    ["Friction coefficient",   "0.90",        "Default — dry asphalt, medium compound"],
    ["Cornering stiffness",    "~8 800 N/rad","dFy/dα at zero slip, 1470 N load"],
    ["Capsize speed",          "~1.9 m/s",    "CBR600RR defaults — stable above this"],
    ["Relaxation length",      "0.30 m",      "Both lateral and longitudinal slip"],
], headers=["Quantity", "Value", "Notes"],
   col_widths=[Inches(2.2), Inches(1.3), Inches(2.7)])

spacer(10)
p_end = doc.add_paragraph()
p_end.alignment = WD_ALIGN_PARAGRAPH.CENTER
r_end = p_end.add_run("End of User Manual — Motorcycle Simulator v3.0 (M3)")
set_font(r_end, size=10, italic=True, color=C_H3)


# ── Save ──────────────────────────────────────────────────────────────────────
doc.save(OUT)
print(f"Saved: {OUT}")
