"""validation/ — virtual road-condition validation suite for two-wheeler sign-off.

Drives the REAL PyChrono physics bike over standardized road profiles (bump, pothole,
cleat, washboard, ISO-8608 rough road, cross-slope) and judges the response against
engineering pass/fail criteria (ISO 2631 ride comfort, suspension bottoming, peak
structural loads, cross-slope stability). Reduces physical prototyping by screening a
design virtually before a build, and exports road-load data for structural/fatigue.
"""
