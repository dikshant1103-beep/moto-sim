"""
core/road/curved_road.py — Oval track with banked corners (Milestone 4).

Track layout (XY plane, Z-up):

    Straight 1:  (0, 0)  → (L, 0)        heading +X,  flat
    Corner 1:    semicircle, centre (L, R)  from (L,0) → (L,2R) via (L+R, R)
    Straight 2:  (L, 2R) → (0, 2R)       heading −X,  flat
    Corner 2:    semicircle, centre (0, R)  from (0,2R) → (0,0)  via (−R, R)

    Travel direction: counter-clockwise when viewed from above.

Banking:
    Straight sections — flat (normal = [0, 0, 1]).
    Corner sections   — road tilts inward by corner_bank [rad]:
        n = sin(b)·n_inward + cos(b)·Z_up
    where n_inward points toward the corner centre.  For banking angle b,
    the normal has unit length regardless of b.

Road height:
    All sections height = 0.  PyChrono's flat ground box (at z = 0) provides
    actual wheel contact; road.height is available in telemetry for future use.

Reference: Cossalter §5 (road-surface forces), VI-grade Road Builder format.
"""

from __future__ import annotations

import math

from .flat_plane import RoadSample


class CurvedRoad:
    """
    Parametric oval track with banked corners.

    query(x, y) finds the nearest point on the four-segment centreline,
    then returns a RoadSample with the correct surface normal and friction.
    """

    def __init__(
        self,
        straight_length: float = 180.0,  # [m] length of each straight
        corner_radius:   float = 10.0,   # [m] radius of each semicircle
        corner_bank:     float = 0.0,    # [rad] inward banking on corners (~0.15 ≈ 8.6°)
        mu_dry:          float = 1.0,
        mu_wet:          float = 0.6,
        surface_type:    str   = "asphalt",
    ) -> None:
        self._L    = straight_length
        self._R    = corner_radius
        self._bank = corner_bank
        self._mu_dry     = mu_dry
        self._mu_wet     = mu_wet
        self._surf       = surface_type

    # ── Public API ───────────────────────────────────────────────────────────
    def query(self, x: float, y: float) -> RoadSample:
        """Return road surface properties at world XY position."""
        normal, height = self._surface(x, y)
        return RoadSample(
            height       = height,
            normal       = normal,
            mu_dry       = self._mu_dry,
            mu_wet       = self._mu_wet,
            surface_type = self._surf,
        )

    @property
    def mu_dry(self) -> float:
        return self._mu_dry

    # ── Internals ────────────────────────────────────────────────────────────
    def _surface(self, x: float, y: float) -> tuple[tuple, float]:
        """
        Return (normal_tuple, height) for position (x, y).

        Segments and their nearest-point projection:
            0 — Straight 1: (0, 0) → (L, 0)
            1 — Straight 2: (L, 2R) → (0, 2R)
            2 — Corner 1:   centre (L, R), angles [−π/2, +π/2]
            3 — Corner 2:   centre (0, R), angles [π/2, 3π/2]
        """
        L, R = self._L, self._R
        flat  = (0.0, 0.0, 1.0)

        # ── Straight 1 ───────────────────────────────────────────────────────
        t1    = max(0.0, min(1.0, x / max(L, 1e-9)))
        px1   = t1 * L
        d1    = math.hypot(x - px1, y)

        # ── Straight 2 ───────────────────────────────────────────────────────
        t2    = max(0.0, min(1.0, (L - x) / max(L, 1e-9)))
        px2   = L - t2 * L
        d2    = math.hypot(x - px2, y - 2.0 * R)

        # ── Corner 1: centre (L, R), right semicircle, θ ∈ [−π/2, π/2] ──────
        dx_c1  = x - L; dy_c1 = y - R
        a1     = math.atan2(dy_c1, dx_c1)
        a1     = max(-math.pi / 2.0, min(math.pi / 2.0, a1))
        px_c1  = L + R * math.cos(a1)
        py_c1  = R + R * math.sin(a1)
        d_c1   = math.hypot(x - px_c1, y - py_c1)
        # Inward unit vector (from contact toward corner centre)
        nin_c1 = (-math.cos(a1), -math.sin(a1))

        # ── Corner 2: centre (0, R), left semicircle, θ ∈ [π/2, 3π/2] ───────
        dx_c2  = x; dy_c2 = y - R
        a2     = math.atan2(dy_c2, dx_c2) % (2.0 * math.pi)  # → [0, 2π)
        a2     = max(math.pi / 2.0, min(3.0 * math.pi / 2.0, a2))
        px_c2  = R * math.cos(a2)
        py_c2  = R + R * math.sin(a2)
        d_c2   = math.hypot(x - px_c2, y - py_c2)
        nin_c2 = (-math.cos(a2), -math.sin(a2))

        # ── Select nearest segment ────────────────────────────────────────────
        seg = min(range(4), key=lambda i: (d1, d2, d_c1, d_c2)[i])

        if seg in (0, 1):
            return flat, 0.0

        # Corner — apply banking
        bank    = self._bank
        sin_b   = math.sin(bank); cos_b = math.cos(bank)
        n_in    = nin_c1 if seg == 2 else nin_c2
        normal  = (sin_b * n_in[0], sin_b * n_in[1], cos_b)
        return normal, 0.0
