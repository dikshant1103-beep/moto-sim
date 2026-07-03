"""run_suite.py — run the full validation battery → pass/fail report + load export.

Drives the real physics bike over every standardized road condition, evaluates each
against the engineering criteria, and writes:

  1. report.json   — per-test pass/fail + metrics (the validation deliverable)
  2. report.csv    — flat table
  3. road_loads_loadcase.json — peak axle road-loads in the Chassis Workbench
                     load-case schema (chassis-workbench/load-case-spec/v1), so the
                     same loads feed CW structural / fatigue analysis (app synergy).

    python validation/run_suite.py --out /tmp/validation
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

sys.path.insert(0, ".")
from core.road.test_roads import standard_test_battery   # noqa: E402
from validation.runner import run_test                   # noqa: E402
from validation.criteria import evaluate                 # noqa: E402


def _kind(name):
    return "cross_slope" if "cross" in name.lower() else "ride"


def run_all(distance=22.0, progress=True):
    results = []
    for name, road, v in standard_test_battery():
        if progress:
            print(f"[validation] running: {name} @ {v} m/s …", flush=True)
        res = run_test(road, target_speed=v, distance=distance)
        ev = evaluate(name, res, kind=_kind(name))
        # attach peak loads for the CW export
        if not res["diverged"]:
            s = res["signals"]
            ev["peak_loads"] = {
                "frontAxle_Fz_N": round(max(s["Fz_f"])),
                "rearAxle_Fz_N": round(max(s["Fz_r"])),
            }
        results.append(ev)
    return results


def to_csv(results):
    rows = ["test,verdict,driver,speed_mps,iso_aw,iso_band,fork_use_pct,shock_use_pct,DAF_front,DAF_rear"]
    for r in results:
        m = r.get("metrics", {})
        iso = m.get("iso2631", {}); su = m.get("suspension", {}); ld = m.get("loads", {})
        rows.append(",".join(str(x) for x in [
            f'"{r["test"]}"', r["verdict"], r.get("driver", ""), r.get("speed", ""),
            iso.get("aw_rms", ""), f'"{iso.get("band","")}"',
            su.get("fork_use_pct", ""), su.get("shock_use_pct", ""),
            ld.get("DAF_front", ""), ld.get("DAF_rear", ""),
        ]))
    return "\n".join(rows)


def to_loadcase_spec(results):
    """Peak axle road-loads as a Chassis Workbench load-case spec (drop-in)."""
    cases = []
    for r in results:
        pl = r.get("peak_loads")
        if not pl:
            continue
        cases.append({
            "def": {"id": r["test"].lower().replace(" ", "_"), "label": r["test"],
                    "kind": "road_event", "color": ""},
            "feasible": r["verdict"] != "fail", "limitedBy": None,
            "Nf": pl["frontAxle_Fz_N"], "Nr": pl["rearAxle_Fz_N"], "leanDeg": 0.0,
            "attachments": [
                {"id": "frontAxle", "label": "Front Axle", "Fx": 0.0, "Fy": 0.0,
                 "Fz": float(pl["frontAxle_Fz_N"]), "resultantF": float(pl["frontAxle_Fz_N"]),
                 "moment": 0.0, "confidence": "computed", "note": "peak road-event vertical load (moto_sim physics)"},
                {"id": "rearAxle", "label": "Rear Axle", "Fx": 0.0, "Fy": 0.0,
                 "Fz": float(pl["rearAxle_Fz_N"]), "resultantF": float(pl["rearAxle_Fz_N"]),
                 "moment": 0.0, "confidence": "computed", "note": "peak road-event vertical load (moto_sim physics)"},
            ],
            "provenance": "moto_sim-validation",
        })
    return {
        "schema": "chassis-workbench/load-case-spec/v1",
        "units": {"force": "N", "moment": "N·m", "frame": "vehicle x-fwd y-right z-up"},
        "meta": {"source": "moto_sim virtual road-validation suite",
                 "note": "peak axle vertical loads from physics ride over standardized road conditions"},
        "cases": cases,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/moto_validation")
    ap.add_argument("--distance", type=float, default=22.0)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)

    results = run_all(distance=a.distance)
    report = {
        "schema": "moto_sim/validation-report/v1",
        "note": ("Virtual road-condition screening. DAF (peak-load) + suspension "
                 "bottoming are robust absolute metrics; ISO-2631 ride number is an "
                 "advisory relative ride-severity index (discrete-contact)."),
        "summary": {v: sum(1 for r in results if r["verdict"] == v) for v in ("pass", "warn", "fail")},
        "tests": results,
    }
    with open(os.path.join(a.out, "report.json"), "w") as f:
        json.dump(report, f, indent=2)
    with open(os.path.join(a.out, "report.csv"), "w") as f:
        f.write(to_csv(results))
    with open(os.path.join(a.out, "road_loads_loadcase.json"), "w") as f:
        json.dump(to_loadcase_spec(results), f, indent=2)

    print("\n══════════ VALIDATION REPORT ══════════")
    for r in results:
        m = r.get("metrics", {}); ld = m.get("loads", {}); su = m.get("suspension", {})
        daf = max(ld.get("DAF_front", 0), ld.get("DAF_rear", 0))
        print(f"  {r['verdict'].upper():4s}  {r['test']:22s}  DAF={daf:.2f}  "
              f"susp={su.get('fork_use_pct','?')}%  (driver: {r.get('driver','')})")
    print(f"  summary: {report['summary']}")
    print(f"  → {a.out}/report.json + report.csv + road_loads_loadcase.json")


if __name__ == "__main__":
    main()
