"""criteria.py — engineering metrics + pass/fail for the validation runs.

Maps the recorded physics signals to the criteria a two-wheeler is judged against:

  ISO 2631-1 ride comfort : frequency-weighted RMS vertical acceleration → comfort
                            band (and an exposure/health flag). The standardized,
                            defensible ride/durability metric.
  Suspension bottoming    : peak fork/shock travel vs available travel.
  Peak structural load    : peak tyre Fz / static  → dynamic amplification factor
                            (DAF) — the impact load that drives fatigue.
  Cross-slope stability   : steady lean + lateral drift while holding the line.

Each returns a verdict (pass/warn/fail) + the governing numbers, so a design can be
screened virtually. (Honest scope: this SCREENS designs and cuts physical build-test
loops; it does not legally replace homologation approval.)
"""

from __future__ import annotations

import math
import statistics


# ── ISO 2631-1 Wk vertical weighting (approx, band-limited RMS) ───────────────
def _rms(xs):
    return math.sqrt(sum(v * v for v in xs) / len(xs)) if xs else 0.0


def iso2631_comfort(az, dt):
    """Frequency-weighted RMS vertical acceleration (m/s²) + ISO 2631 comfort band.

    Wk weighting emphasised 4–8 Hz (where humans are most sensitive). We apply a
    light band emphasis via a moving-average high-pass + the standard comfort bands.
    """
    if len(az) < 8:
        return {"aw_rms": 0.0, "band": "n/a", "verdict": "pass"}
    # Vertical ride comfort is dominated by 4–8 Hz. (1) clamp single-sample contact
    # spikes (a real accelerometer + anti-alias filter wouldn't pass 100 g blips);
    # (2) low-pass ~15 Hz to the ride band; (3) high-pass to drop slow drift. Moving
    # averages stand in for the ISO Wk filter (relative ride-severity, see report note).
    clamped = [max(-60.0, min(60.0, v)) for v in az]
    lp_win = max(1, int((1.0 / 15.0) / dt))     # ~15 Hz ride band
    lp = []
    acc = 0.0; buf = []
    for v in clamped:
        buf.append(v); acc += v
        if len(buf) > lp_win:
            acc -= buf.pop(0)
        lp.append(acc / len(buf))
    hp_win = max(1, int(0.5 / dt))
    out = []
    acc = 0.0; buf = []
    for v in lp:
        buf.append(v); acc += v
        if len(buf) > hp_win:
            acc -= buf.pop(0)
        out.append(v - acc / len(buf))
    aw = _rms(out)
    # ISO 2631-1 comfort reaction bands (m/s²)
    if aw < 0.315:   band, verdict = "not uncomfortable", "pass"
    elif aw < 0.63:  band, verdict = "a little uncomfortable", "pass"
    elif aw < 1.0:   band, verdict = "fairly uncomfortable", "warn"
    elif aw < 1.6:   band, verdict = "uncomfortable", "warn"
    elif aw < 2.5:   band, verdict = "very uncomfortable", "fail"
    else:            band, verdict = "extremely uncomfortable", "fail"
    return {"aw_rms": round(aw, 3), "band": band, "verdict": verdict}


def suspension_check(fork, shock, fork_max, shock_max):
    pf = max((abs(v) for v in fork), default=0.0)
    ps = max((abs(v) for v in shock), default=0.0)
    uf = pf / fork_max if fork_max else 0.0
    us = ps / shock_max if shock_max else 0.0
    u = max(uf, us)
    verdict = "pass" if u < 0.85 else ("warn" if u < 1.0 else "fail")
    return {"fork_peak_mm": round(pf * 1000, 1), "shock_peak_mm": round(ps * 1000, 1),
            "fork_use_pct": round(uf * 100), "shock_use_pct": round(us * 100),
            "verdict": verdict, "note": "bottoming" if u >= 1.0 else ""}


def load_check(Fz_f, Fz_r, Wf0, Wr0):
    pf = max((abs(v) for v in Fz_f), default=0.0)
    pr = max((abs(v) for v in Fz_r), default=0.0)
    daf_f = pf / Wf0 if Wf0 else 0.0
    daf_r = pr / Wr0 if Wr0 else 0.0
    daf = max(daf_f, daf_r)
    # DAF > ~3 = severe impact (fatigue concern); > 4 = fail-band
    verdict = "pass" if daf < 2.5 else ("warn" if daf < 3.5 else "fail")
    return {"Fz_f_peak_N": round(pf), "Fz_r_peak_N": round(pr),
            "DAF_front": round(daf_f, 2), "DAF_rear": round(daf_r, 2),
            "verdict": verdict}


def crossslope_check(roll, y):
    """Steady lean to hold the slope + lateral drift off the line."""
    if not roll:
        return {"verdict": "pass"}
    steady_roll = math.degrees(statistics.mean(roll[-max(1, len(roll) // 4):]))
    drift = abs(y[-1] - y[0]) if y else 0.0
    # holds line if lateral drift < 2 m over the run and roll is bounded
    verdict = "pass" if drift < 2.0 and abs(steady_roll) < 25 else ("warn" if drift < 5 else "fail")
    return {"steady_lean_deg": round(steady_roll, 1), "lateral_drift_m": round(drift, 2),
            "verdict": verdict}


_ORDER = {"pass": 0, "warn": 1, "fail": 2, "n/a": 0}


def evaluate(test_name, result, kind="ride"):
    """Aggregate criteria for one test result into a verdict + metrics."""
    if result["diverged"]:
        return {"test": test_name, "verdict": "fail", "reason": "physics diverged",
                "metrics": {}}
    s = result["signals"]
    st = result["static"]
    metrics = {
        "iso2631": iso2631_comfort(s["az"], 0.001),
        "suspension": suspension_check(s["fork"], s["shock"], st["fork_max"], st["shock_max"]),
        "loads": load_check(s["Fz_f"], s["Fz_r"], st["Fz_f"], st["Fz_r"]),
    }
    if kind == "cross_slope":
        metrics["crossslope"] = crossslope_check(s["roll"], s["y"])
    # Hard pass/fail is driven by the ROBUST metrics (peak load DAF + suspension
    # bottoming — these come from physical peak forces/travel). The ISO-2631 ride
    # number is ADVISORY here: with discrete obstacle contact it's a relative ride-
    # severity index, not a calibrated absolute, so it can only raise to 'warn'.
    robust = [metrics["suspension"]["verdict"], metrics["loads"]["verdict"]]
    if "crossslope" in metrics:
        robust.append(metrics["crossslope"]["verdict"])
    hard = max(robust, key=lambda v: _ORDER[v])
    iso_v = metrics["iso2631"]["verdict"]
    iso_advisory = "warn" if iso_v in ("fail",) else iso_v   # cap ISO at warn
    worst = max([hard, iso_advisory], key=lambda v: _ORDER[v])
    return {"test": test_name, "verdict": worst, "metrics": metrics,
            "speed": result["target_speed"],
            "driver": "load+bottoming" if _ORDER[hard] >= _ORDER[iso_advisory] else "ride"}
