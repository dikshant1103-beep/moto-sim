"""validation/ — virtual road-condition validation suite for two-wheeler sign-off.

Drives the REAL PyChrono physics bike over standardized road profiles (bump, pothole,
cleat, washboard, ISO-8608 rough road, cross-slope) and judges the response against
engineering pass/fail criteria (ISO 2631 ride comfort, suspension bottoming, peak
structural loads, cross-slope stability). Reduces physical prototyping by screening a
design virtually before a build, and exports road-load data for structural/fatigue.

Sub-packages
────────────
probes/ — model-correctness probes kept as a permanent record. These found and
    fixed a sign inversion in the front-tyre slip kinematics (the motor angle is
    the negative of the geometric steer angle) and disproved a long-standing
    claim that PyChrono's constraint chain failed to route the tyre aligning
    moment to the steer axis. See validation/probes/STEER_SIGN_AUDIT.md and
    docs/decisions/006-steer-sign-inversion.md.
"""
