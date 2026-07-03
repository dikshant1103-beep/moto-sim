# Positioning — Two-Wheeler Virtual Development Platform

*Working title:* **MotoForge** (placeholder) — an integrated, laptop-class virtual
development & screening platform for two-wheeler / EV-two-wheeler design.

*Status:* working prototype across two apps (moto_sim physics + Chassis Workbench
concept tool). This document defines what it is, who it's for, what it honestly is
and isn't, and the path forward. It is both the **product positioning** and the
**portfolio / research narrative**.

Last updated: 2026-06-29.

---

## 1. One-line positioning

> **The early-stage virtual workbench for two-wheeler teams who can't afford
> Adams/SIMPACK — concept, target-setting, and durability/ride *screening* on a
> laptop, with a digital-twin/AI testing layer.**

We are **not** a homologation/certification tool and we do **not** replace physical
prototyping. We **reduce the number of physical build-test iterations** by catching
bad designs early, and we **feed the load cases / scenarios** that the expensive
validated CAE tools then take to sign-off.

---

## 2. The problem (and who actually feels it)

Designing a two-wheeler chassis/suspension today splits into two worlds:

- **The incumbents** (Adams/Car, SIMPACK, IPG Motorcyclemaker, AVL): validated,
  trusted, certification-grade — and **six-figure licenses + dedicated CAE teams**.
  Owned by large OEMs and tier-1s.
- **Everyone else**: SME two-wheeler makers, **EV-two-wheeler startups**, custom
  builders, and engineering programs. They **cannot afford that stack** and **don't
  have CAE teams**, so they design by **physical trial-and-error** — build a
  prototype, ride it, find the suspension bottoms out / the frame's too soft / the
  weight balance is wrong, and **rebuild.** Every iteration is weeks and lakhs.

The pain is concentrated in the **budget segment**, and it's growing: the global —
and especially Indian — **EV-two-wheeler startup wave** (Ola, Ather, Ultraviolette
and a long tail of smaller players) means more teams designing two-wheelers than ever,
most of them *without* the CAE budget the incumbents assume.

**Nobody serves the early funnel for the budget segment.** That's the gap.

---

## 3. The honest truth about "virtual validation" (so we stay credible)

A simulation reduces physical prototyping **only when it has been correlated against
real test data and trusted enough to skip a build.** That trust — not the math — is
the entire moat of the incumbents (decades of physical correlation).

Therefore:
- **We will not claim to replace homologation or physical prototyping.** Any such
  claim is false and would destroy credibility on first contact with a real engineer.
- **What we honestly offer is *screening*:** catching obviously-bad designs before
  they're built, and reducing the *number* of physical iterations.
- **Our credibility rests on transparency:** every number is labelled *screening-grade*,
  *theory-correlated*, or *estimated* — never *validated* unless it is.

This honesty is a feature, not a weakness. It's what lets a budget team trust us for
the decisions we *are* good for, instead of distrusting us for over-claiming.

---

## 4. What it is / what it isn't

| It IS | It is NOT |
|-------|-----------|
| Concept & target-setting workbench (geometry, mass, stiffness targets, load cases) | A certification / homologation authority |
| Durability & ride **screening** (bumps, cleats, rough road → loads, bottoming, ride) | A replacement for Adams/SIMPACK detailed CAE |
| A **pre-processor** that generates load cases / scenarios for real CAE | A flexible-multibody / FEA solver |
| A digital-twin / AI testing layer (ROS 2, Gazebo, SLAM, perception) | Validated against measured physical-test data (yet) |
| Laptop-class, low-cost, integrated | A point tool for one narrow analysis |

---

## 5. The wedge (target customer)

**Primary:** budget **EV-two-wheeler startups & SME OEMs** (1–50 engineers, no CAE
license budget) who today iterate physically. We help them kill bad concepts on a
laptop before cutting metal.

**Secondary:** **engineering education & upskilling** (mechanical/automotive programs,
EV-skilling bootcamps) — the integrated, visual, AI-enabled platform is a strong
teaching tool, a lower-credibility-bar entry market, and a funnel for future buyers.

**Why these:** they have the pain, no incumbent serves them, and the fidelity we can
honestly deliver (screening) matches the decisions they actually need to make early.

**Geography:** India-first (largest two-wheeler market + booming EV-2W startup scene +
cost sensitivity + education scale), then other emerging two-wheeler markets.

---

## 6. Competitive landscape & where we fit

```
 fidelity / cost
   ^
   |   Adams/Car · SIMPACK · IPG Motorcyclemaker · AVL   (certification CAE)
   |        ▲ hand off load cases / scenarios UP to these
   |        │
   |   ┌────┴─────────────────────────┐
   |   │  US: integrated screening +  │   ← the gap nobody fills
   |   │  concept + digital-twin/AI   │
   |   └──────────────────────────────┘
   |        │ feeds / teaches DOWN from these
   |        ▼
   |   spreadsheets · hand calcs · physical trial-and-error
   +------------------------------------------------------> breadth / accessibility
```

We **complement** the incumbents (we're the cheap front-end that feeds them), and we
**replace** spreadsheets + blind physical iteration for the budget segment. We never
fight the incumbents on certification fidelity — we'd lose, and it's not the job.

---

## 7. The product (what is actually built today)

Two integrated apps form the pipeline:

**A. Chassis Workbench** — the concept / target lane (32 tabs):
- Geometry, mass/CoG, suspension, anti-squat/anti-dive, ergonomics, stability.
- **Stiffness Targets** (derive required frame torsional/lateral stiffness).
- **Load-Case Generator** (interface loads at the attachment points → CAE spec export).
- Python physics backend; structural Gazebo rig lab.

**B. moto_sim** — the dynamics & digital-twin lane:
- **Real PyChrono multibody** motorcycle (suspension, tire, balance).
- **Virtual road-validation suite**: bumps, cleats, washboard, ISO-8608 rough road →
  ISO-2631 ride index, suspension bottoming, peak-load DAF, pass/fail report.
- Road-loads **export in the Chassis Workbench load-case schema** (the two apps share
  one data spine — design loads flow moto_sim → Chassis Workbench → CAE).
- **Digital-twin / AI testing layer**: ROS 2 + Gazebo city/oval worlds, SLAM, onboard
  camera + lidar, YOLO obstacle perception, Qwen high-level decisions, autonomous ride.
- **SPSA gain auto-tuning** (stochastic-gradient optimization of the controller).

This breadth — *concept → targets → load cases → durability/ride screening →
digital-twin/AI* in one accessible stack — **is the differentiator.** The incumbents
are fragmented point tools; an integrated accessible pipeline is a real, defensible angle.

---

## 8. Credibility anchor (the floor)

The platform's numbers are not invented — the core suspension dynamics are
**correlated to established motorcycle behavior on three levels**:

| Check | Sim | Reference | |
|-------|-----|-----------|---|
| Body-bounce frequency | **1.37 Hz** | published 1.0–2.0 Hz (Cossalter/Foale) | ✓ |
| Suspension damping ζ | **0.45** | published 0.2–0.6 | ✓ |
| Static sag (front / rear) | **38 / 75 mm** | real-bike 25–45 / 55–95 mm | ✓ |
| **Foale sag-law** `fn=(1/2π)√(g/δ)` | rear 75 mm → **1.82 Hz** | Foale anchors 62 mm↔2.0 Hz, 110 mm↔1.5 Hz → 75 mm sits between | ✓ |

The strongest point: the sim's **measured static sag**, fed into **Foale's published
sag-frequency law**, lands exactly where Foale's worked anchor values say it should —
and the larger-than-nominal rear sag reveals the sim correctly captures **swingarm
motion-ratio softening** (effective rear wheel-rate 13.4 kN/m vs nominal 24 kN/m). That
is verified physical fidelity, not a tuned number.

**Independent longitudinal benchmarks** (exact closed-form, separate from the
vibration/stability anchors above):

| Benchmark | Sim | Theory | |
|-----------|-----|--------|---|
| Static axle load | 46.0% front | x_cg/L = 46.0% | ✓ 0.0% |
| Braking load transfer (3 pts) | 43/92/140 N | m·a·h/L = 44/93/141 N | ✓ 1.1% |

**Stability reference:** the **Whipple/Meijaard (2007) benchmark** — the canonical
single-track stability reference — is reproduced to **0.02%** (weave 4.293 vs 4.292,
capsize 6.025 vs 6.024 m/s).

So the credibility rests on **four independent anchors**: suspension ride dynamics,
the Foale sag-law, longitudinal load transfer, and the Whipple stability benchmark —
spanning vertical, longitudinal, and lateral dynamics.

**Honest scope:** these correlate to **published laws, worked values, and the canonical
benchmark** (a strong, multi-axis credibility *floor*), **not** yet to a specific
*instrumented physical-test dataset*. That single physical-data correlation is the
roadmap's first commercial milestone. Known sim limits stated openly (e.g. longitudinal
grip saturates ~0.9 g, so it can't reach a 1.28 g stoppie).

---

## 9. Why us / differentiation

1. **Integrated breadth** — one pipeline vs the incumbents' fragmented point tools.
2. **Accessibility & cost** — runs on a laptop (GTX 1650 Ti class), no license wall.
3. **EV-2W + AI/digital-twin modernity** — ROS/Gazebo/SLAM/perception autonomous-testing
   layer no budget-segment competitor offers; ties to adjacent EV work (battery/digital-twin).
4. **Radical honesty about fidelity** — builds trust with real engineers where over-claiming
   competitors lose it.

---

## 10. Business model (initial hypotheses)

- **SaaS / seat licence** for SME OEMs & startups — priced an order of magnitude below
  incumbents (target: affordable on a startup eng budget, not a CAE-team budget).
- **Education tier** — cheap/free for institutions; funnel + brand + future buyers.
- **Services / correlation-as-a-service** — help a customer correlate the sim to *their*
  bike for *their* narrow use case (turns the credibility gap into revenue).
- **Pipeline / pre-processor licence** — sell the "feeds your existing CAE" angle to
  slightly larger teams who already own Adams/ANSYS but want a cheaper front-end.

---

## 11. Roadmap (honest stages)

**Now (built):** integrated concept + screening + digital-twin/AI pipeline; theory
correlation floor.

**Near (credibility + packaging):**
1. **One physical correlation point** — correlate the sim against one real or
   published *measured* result (e.g., a known bump-response or stability case). This
   is the single highest-leverage commercial step.
2. **Tighten the moto_sim → Chassis Workbench → CAE handoff** into one demo flow.
3. **Validation dashboard** — run the screening battery + pass/fail report from the UI.

**Mid (fidelity where it pays):**
4. Faithful pothole (drop-in) + cross-slope lateral dynamics (current weak tests).
5. Calibrated ISO-2631 ride index (proper Wk filter).
6. Frame-flexibility / fatigue post-processing (rainflow + S-N) — the durability story.

**Long (trust at scale):** a library of correlation cases across vehicle classes;
the asset that lets customers trust skipping iterations.

---

## 12. Honest limitations (state these openly)

- Simplified multibody (rigid frame; straight-line-stable controller) — good for
  concept/screening, not detailed CAE.
- Validation suite: load-DAF + bottoming are robust; **ISO-2631 ride number is an
  advisory relative index**; **pothole is approximated** (boxes can't dig a hole);
  **cross-slope lateral dynamics not yet faithful**.
- Correlation is **theory/literature only** so far.
- Single-developer prototype; not productized/hardened.

None of these block the *screening* value proposition for the budget segment — and
naming them is what keeps us credible.

---

## 13. Portfolio / research framing

Even before any commercialization, this is a strong standalone story:

> *"An integrated, laptop-class virtual development platform for two-wheelers
> combining real multibody physics, a road-condition validation suite correlated to
> published motorcycle dynamics, and a ROS 2 / Gazebo / AI digital-twin testing layer
> (SLAM, perception, autonomous ride, stochastic-gradient control tuning) — built
> solo, demonstrating breadth across vehicle dynamics, robotics, and ML."*

That sentence is true today and demonstrates exactly the cross-disciplinary capability
(dynamics + robotics + ML + honest engineering judgment) that the work has produced.
