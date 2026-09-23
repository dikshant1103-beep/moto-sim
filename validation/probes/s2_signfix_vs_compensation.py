#!/usr/bin/env python3
"""
S2 — Is K_Mz_missing compensating a constraint failure, or masking a sign bug?

S1/S1b/S1c established that:
  * the aligning moment Mz DOES reach the steering axis (|eta| = 0.96),
  * the front-tyre slip kinematics evaluate the wheel at the NEGATIVE of its
    true geometric steer angle.

If the second finding is the real cause of the steer drift that motivated
K_Mz_missing = 2000 N.m/rad, then correcting the sign should let the
compensation be removed without losing stability.

This script runs the full 2 x 6 x 3 matrix

    steer_kin_sign  in {+1 (legacy), -1 (corrected)}
    K_Mz_missing    in {0, 250, 500, 1000, 2000, 3000} N.m/rad
    speed           in {5, 6, 10} m/s

with the standard 5 deg roll perturbation, and reports peak steer excursion,
peak roll, and roll sign reversals. Steer is measured GEOMETRICALLY so the
comparison is unaffected by the motor sign convention.

Usage:  python validation/probes/s2_signfix_vs_compensation.py
Output: validation/probes/data/s2_signfix.json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from harness import DATA, build, metrics, run  # noqa: E402

SPEEDS = [5.0, 6.0, 10.0]
KMZ = [0.0, 250.0, 500.0, 1000.0, 2000.0, 3000.0]
SIGNS = [(+1.0, "legacy"), (-1.0, "corrected")]
SIM_S = 3.0
PERTURB = 5.0
K_YAW_DAMP = 2000.0   # pinned: pre-fix value, see comment in main()


def main() -> None:
    rows = []
    print(f"{'kin':<10} {'K_Mz':>6} {'v':>5} {'stable':>7} "
          f"{'maxRoll':>8} {'maxSteer':>9} {'rev':>4}")
    print("-" * 56)
    for sign, label in SIGNS:
        for kmz in KMZ:
            for v in SPEEDS:
                # K_yaw_damp pinned to its pre-fix value so this sweep varies
                # exactly one thing (K_Mz_missing) and stays reproducible as
                # model defaults change.
                m = build(v0=v, steer_kin_sign=sign, K_Mz_missing=kmz,
                          K_yaw_damp=2000.0, perturb_deg=PERTURB)
                mt = metrics(run(m, SIM_S))
                rows.append({"kin": label, "steer_kin_sign": sign,
                             "K_Mz_missing": kmz, "v": v, **mt})
                print(f"{label:<10} {kmz:6.0f} {v:5.1f} {str(mt['stable']):>7} "
                      f"{mt['max_abs_roll_deg']:8.2f} "
                      f"{mt['max_abs_steer_deg']:9.3f} "
                      f"{mt['roll_reversals']:4d}")

    DATA.mkdir(parents=True, exist_ok=True)
    out = DATA / "s2_signfix.json"
    out.write_text(json.dumps({"sim_s": SIM_S, "perturb_deg": PERTURB,
                               "rows": rows}, indent=2))
    print(f"\nWrote {out}")


if __name__ == "__main__":
    main()
