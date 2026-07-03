"""test_roads.py — standardized road-condition profiles for virtual validation.

Each profile implements the road interface  query(x, y) -> RoadSample  (height +
unit normal + friction), exactly like FlatPlane/CurvedRoad, so the EXISTING physics
consumes them with no engine change (motorcycle.py calls road.query() each step for
both wheels and feeds height/normal into the tire/suspension).

These reproduce the road conditions a two-wheeler must survive for approval /
durability sign-off, so a design can be screened virtually before a physical build:

  discrete obstacles : BumpRoad, PotholeRoad, CleatRoad
  rough road (ride)  : WashboardRoad, ISO8608Road  (random roughness, classes A–E)
  road lean angle    : CrossSlopeRoad  (laterally tilted surface / camber)
  banked / mixed     : (CrossSlope with along-x tilt, or core/road/curved_road.py)

Units: m, rad. Coordinate: x forward, y left, z up. Bike rides toward +x.
"""

from __future__ import annotations

import math
import random

from core.road.flat_plane import RoadSample


def _normal_from_grads(dz_dx: float, dz_dy: float):
    """Unit surface normal for a height field z(x,y) with the given gradients."""
    nx, ny, nz = -dz_dx, -dz_dy, 1.0
    n = math.sqrt(nx * nx + ny * ny + nz * nz)
    return (nx / n, ny / n, nz / n)


class BumpRoad:
    """One or more raised bumps (half-sine speed-bump / hump) across the lane.

    A classic durability + ride event. Bump height `h`, length `L` (along x),
    centered at each x in `centers`.
    """
    def __init__(self, centers=(20.0,), height=0.05, length=0.6, mu_dry=1.0):
        self.centers = list(centers)
        self.h = height
        self.L = length
        self.mu = mu_dry

    def _profile(self, x):
        z = 0.0; dz = 0.0
        for c in self.centers:
            if abs(x - c) <= self.L / 2:
                # half-sine hump: z = h·cos(π(x−c)/L) on [−L/2, L/2]
                z = self.h * math.cos(math.pi * (x - c) / self.L)
                dz = -self.h * math.pi / self.L * math.sin(math.pi * (x - c) / self.L)
        return z, dz

    def query(self, x, y):
        z, dz = self._profile(x)
        return RoadSample(height=z, normal=_normal_from_grads(dz, 0.0),
                          mu_dry=self.mu, mu_wet=self.mu * 0.65, surface_type="bump")


class PotholeRoad:
    """A depression (pothole) with a sharp leading edge — high impact load test."""
    def __init__(self, center=20.0, depth=0.06, length=0.5, mu_dry=1.0):
        self.c = center; self.d = depth; self.L = length; self.mu = mu_dry

    def query(self, x, y):
        z = 0.0; dz = 0.0
        if abs(x - self.c) <= self.L / 2:
            # smooth dip:  z = -d * (1 + cos(2π(x-c)/L)) / 2
            ph = 2 * math.pi * (x - self.c) / self.L
            z = -self.d * (1 + math.cos(ph)) / 2
            dz = self.d * math.pi / self.L * math.sin(ph)
        return RoadSample(height=z, normal=_normal_from_grads(dz, 0.0),
                          mu_dry=self.mu, mu_wet=self.mu * 0.65, surface_type="pothole")


class CleatRoad:
    """Rectangular cleat(s) — the standard square-edge impact durability obstacle.

    Sharp rise/fall → worst-case impact load into tire/suspension/frame.
    """
    def __init__(self, centers=(20.0,), height=0.025, width=0.05, mu_dry=1.0):
        self.centers = list(centers); self.h = height; self.w = width; self.mu = mu_dry

    def query(self, x, y):
        z = 0.0
        for c in self.centers:
            if abs(x - c) <= self.w / 2:
                z = self.h
        # near-vertical edges → use a steep but finite gradient for the normal
        dz = 0.0
        for c in self.centers:
            if abs(abs(x - c) - self.w / 2) < 0.01:
                dz = (self.h / 0.01) * (1 if x < c else -1)
        return RoadSample(height=z, normal=_normal_from_grads(dz, 0.0),
                          mu_dry=self.mu, mu_wet=self.mu * 0.65, surface_type="cleat")


class WashboardRoad:
    """Periodic sinusoidal corrugation — sustained vibration input for ride/comfort."""
    def __init__(self, start=10.0, amplitude=0.015, wavelength=0.8, length=60.0, mu_dry=1.0):
        self.x0 = start; self.a = amplitude; self.lam = wavelength
        self.x1 = start + length; self.mu = mu_dry

    def query(self, x, y):
        z = 0.0; dz = 0.0
        if self.x0 <= x <= self.x1:
            k = 2 * math.pi / self.lam
            z = self.a * math.sin(k * (x - self.x0))
            dz = self.a * k * math.cos(k * (x - self.x0))
        return RoadSample(height=z, normal=_normal_from_grads(dz, 0.0),
                          mu_dry=self.mu, mu_wet=self.mu * 0.7, surface_type="washboard")


class ISO8608Road:
    """Random road roughness per ISO 8608 (road classes A=smooth … E=very rough).

    Builds a 1-D longitudinal profile by summing sinusoids whose amplitudes follow
    the ISO 8608 PSD  Gd(n) = Gd(n0)·(n/n0)^−2,  n0 = 0.1 cyc/m. Gd(n0) sets the
    class: A 1e-6, B 4e-6, C 16e-6, D 64e-6, E 256e-6  [m³/cyc]. Deterministic per seed.
    """
    CLASS_GD0 = {"A": 1e-6, "B": 4e-6, "C": 16e-6, "D": 64e-6, "E": 256e-6}

    def __init__(self, road_class="C", start=10.0, length=80.0, seed=1, n_harm=200, mu_dry=1.0):
        self.x0 = start; self.x1 = start + length; self.mu = mu_dry
        Gd0 = self.CLASS_GD0.get(road_class.upper(), 16e-6)
        rng = random.Random(seed)
        n0 = 0.1
        # spatial frequencies from 0.01 to 10 cyc/m (log-spaced)
        self._terms = []  # (amp, k, phase)
        nlo, nhi = 0.02, 4.0
        for i in range(n_harm):
            n = nlo * (nhi / nlo) ** (i / (n_harm - 1))           # cyc/m
            dn = n * math.log(nhi / nlo) / (n_harm - 1)
            Gd = Gd0 * (n / n0) ** -2
            amp = math.sqrt(2 * Gd * dn)                           # m
            self._terms.append((amp, 2 * math.pi * n, rng.uniform(0, 2 * math.pi)))
        self.road_class = road_class.upper()

    def _z(self, x):
        s = x - self.x0
        return sum(a * math.sin(k * s + ph) for a, k, ph in self._terms)

    def query(self, x, y):
        if not (self.x0 <= x <= self.x1):
            return RoadSample(height=0.0, normal=(0, 0, 1), mu_dry=self.mu, surface_type="iso8608")
        z = self._z(x)
        dz = (self._z(x + 1e-3) - z) / 1e-3
        return RoadSample(height=z, normal=_normal_from_grads(dz, 0.0),
                          mu_dry=self.mu, mu_wet=self.mu * 0.7, surface_type="iso8608")


class CrossSlopeRoad:
    """Laterally tilted road surface — the road-lean-angle / cross-slope (camber) test.

    The surface tilts about the x-axis by `slope_deg` (positive → tilts down to the
    right, −y). The bike must lean/steer to hold the line; the tilted NORMAL feeds the
    contact so the engine sees the cross-slope. Constant after `start`.
    """
    def __init__(self, slope_deg=6.0, start=8.0, mu_dry=1.0):
        self.beta = math.radians(slope_deg); self.x0 = start; self.mu = mu_dry
        self._tan = math.tan(self.beta)

    def query(self, x, y):
        if x < self.x0:
            return RoadSample(height=0.0, normal=(0, 0, 1), mu_dry=self.mu, surface_type="cross_slope")
        # surface drops toward −y: z = −y*tan(beta) → dz/dy = −tan(beta)
        z = -y * self._tan
        return RoadSample(height=z, normal=_normal_from_grads(0.0, -self._tan),
                          mu_dry=self.mu, mu_wet=self.mu * 0.7, surface_type="cross_slope")


# Registry the runner/report iterate over. Speeds are typical test speeds (m/s).
def standard_test_battery():
    return [
        ("Speed bump (50mm)",      BumpRoad(centers=(20,), height=0.05, length=0.6),   5.0),
        ("Pothole (60mm)",         PotholeRoad(center=20, depth=0.06, length=0.5),     6.0),
        ("Square cleat (25mm)",    CleatRoad(centers=(20,), height=0.025, width=0.05), 8.0),
        ("Washboard road",         WashboardRoad(amplitude=0.015, wavelength=0.8),     8.0),
        ("Rough road ISO-8608 C",  ISO8608Road(road_class="C"),                        10.0),
        ("Rough road ISO-8608 D",  ISO8608Road(road_class="D"),                        8.0),
        ("Cross-slope 6deg",       CrossSlopeRoad(slope_deg=6.0),                       8.0),
    ]
