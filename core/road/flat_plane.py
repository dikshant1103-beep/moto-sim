"""
core/road/flat_plane.py — Infinite flat road for Milestone 1.

Provides the road query interface that all road implementations must satisfy.
Milestone 4 will provide a parametric track implementing the same interface.

Road query interface
────────────────────
    road.query(x, y) → RoadSample

RoadSample contains:
    height  [m]     road surface Z at (x, y)
    normal  (nx, ny, nz) — unit normal (0, 0, 1) for flat plane
    mu_dry  [–]     dry-surface friction coefficient
    mu_wet  [–]     wet-surface friction coefficient (for M5 rainfall)
    surface_type    string label

All road implementations must return a RoadSample.  The motorcycle queries
one sample per wheel per physics step.  No file I/O in the hot path.
"""

from __future__ import annotations

from dataclasses import dataclass


# ─────────────────────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class RoadSample:
    """Result of a road surface query at a single point."""
    height:       float = 0.0
    normal:       tuple = (0.0, 0.0, 1.0)   # (nx, ny, nz), Z-up
    mu_dry:       float = 1.0
    mu_wet:       float = 0.6
    surface_type: str   = "asphalt"


# ─────────────────────────────────────────────────────────────────────────────
class FlatPlane:
    """
    Infinite flat asphalt road at Z = 0.

    Milestone 1 default road.  All queries return the same RoadSample.
    """

    def __init__(
        self,
        mu_dry: float = 1.0,
        mu_wet: float = 0.6,
        height: float = 0.0,
    ) -> None:
        self._sample = RoadSample(
            height=height,
            normal=(0.0, 0.0, 1.0),
            mu_dry=mu_dry,
            mu_wet=mu_wet,
            surface_type="asphalt",
        )

    def query(self, x: float, y: float) -> RoadSample:
        """Return road properties at (x, y).  O(1), no I/O."""
        return self._sample

    @property
    def mu_dry(self) -> float:
        return self._sample.mu_dry
