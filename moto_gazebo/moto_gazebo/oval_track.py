"""oval_track.py — shared oval race-track geometry: centerline sampling + world SDF.

Used by BOTH the world generator (draw the track) and path_driver (follow the
centerline). One source of truth so the bike rides exactly on the road.

Oval centerline (x fwd, y left), built from two straights + two semicircle ends:
  Straight 1 : (0..Ls, 0)            heading +x
  Corner 1   : centre (Ls, Rc)       angle -pi/2 → +pi/2   (right-hand U)
  Straight 2 : (Ls..0, 2Rc)          heading -x
  Corner 2   : centre (0,  Rc)       angle +pi/2 → +3pi/2

It is a closed loop of total length  perim = 2*Ls + 2*pi*Rc.
sample(s) returns (x, y, heading, curvature) at arc-length s (wraps around).
"""

from __future__ import annotations

import math

DEFAULT_LS = 120.0   # straight length [m]
DEFAULT_RC = 45.0    # corner radius [m]
ROAD_W = 10.0        # road width [m]


def perim(Ls=DEFAULT_LS, Rc=DEFAULT_RC):
    return 2.0 * Ls + 2.0 * math.pi * Rc


def sample(s, Ls=DEFAULT_LS, Rc=DEFAULT_RC):
    """Position + heading + curvature at arc length s along the oval centerline."""
    P = perim(Ls, Rc)
    s = s % P
    corner_len = math.pi * Rc
    # Segment 1: straight 1  (0..Ls), heading +x, y=0
    if s < Ls:
        return (s, 0.0, 0.0, 0.0)
    s -= Ls
    # Segment 2: corner 1, centre (Ls, Rc), angle -pi/2 → +pi/2
    if s < corner_len:
        a = -math.pi / 2 + s / Rc
        x = Ls + Rc * math.cos(a)
        y = Rc + Rc * math.sin(a)
        heading = a + math.pi / 2          # tangent (CCW)
        return (x, y, heading, 1.0 / Rc)
    s -= corner_len
    # Segment 3: straight 2  (Ls..0), heading +pi (−x), y=2Rc
    if s < Ls:
        return (Ls - s, 2.0 * Rc, math.pi, 0.0)
    s -= Ls
    # Segment 4: corner 2, centre (0, Rc), angle +pi/2 → +3pi/2
    a = math.pi / 2 + s / Rc
    x = Rc * math.cos(a)
    y = Rc + Rc * math.sin(a)
    heading = a + math.pi / 2
    return (x, y, heading, 1.0 / Rc)


def _centerline(Ls, Rc, ds=4.0):
    pts = []
    s = 0.0
    P = perim(Ls, Rc)
    while s < P:
        pts.append(sample(s, Ls, Rc))
        s += ds
    return pts


_WORLD_HEADER = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="oval_track">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_update_rate>250</real_time_update_rate></physics>
    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"><ros><namespace>/moto</namespace></ros></plugin>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so"><ros><namespace>/moto</namespace></ros><update_rate>100</update_rate></plugin>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.85 1.0 1</background><shadows>true</shadows></scene>
    <include><uri>model://sun</uri></include>
    <model name="ground"><static>true</static>
      <link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry>
          <material><ambient>0.20 0.45 0.18 1</ambient><diffuse>0.20 0.45 0.18 1</diffuse></material></visual>
      </link></model>
"""
_WORLD_FOOTER = "  </world>\n</sdf>\n"


def generate_oval_world(Ls=DEFAULT_LS, Rc=DEFAULT_RC, road_w=ROAD_W) -> str:
    """Clean oval circuit: continuous asphalt ribbon (flat, on the centerline the bike
    rides) + red/white kerbs on BOTH edges + a start/finish line. All landmark buildings
    are placed WELL CLEAR of the track (infield centre + far outside) so nothing sits on
    the bike's path."""
    pts = _centerline(Ls, Rc, ds=3.0)
    parts = [_WORLD_HEADER]
    parts.append('    <model name="track"><static>true</static><link name="l">')
    seg = 3.4  # > ds (3.0) → continuous overlap, no gaps
    for i, (x, y, h, _) in enumerate(pts):
        # asphalt
        parts.append(
            f'<visual name="r{i}"><pose>{x:.2f} {y:.2f} 0.02 0 0 {h:.4f}</pose>'
            f'<geometry><box><size>{seg:.2f} {road_w:.2f} 0.04</size></box></geometry>'
            f'<material><ambient>0.15 0.15 0.17 1</ambient><diffuse>0.17 0.17 0.19 1</diffuse></material></visual>')
        # kerbs both edges (perp = (-sin h, cos h))
        col = "0.85 0.12 0.12" if i % 2 == 0 else "0.92 0.92 0.92"
        off = road_w / 2 + 0.5
        for side in (+1, -1):
            kx = x + (-math.sin(h)) * off * side
            ky = y + (math.cos(h)) * off * side
            parts.append(
                f'<visual name="k{i}_{side}"><pose>{kx:.2f} {ky:.2f} 0.05 0 0 {h:.4f}</pose>'
                f'<geometry><box><size>{seg:.2f} 0.5 0.05</size></box></geometry>'
                f'<material><ambient>{col} 1</ambient><diffuse>{col} 1</diffuse></material></visual>')
        # centre dashes
        if i % 4 == 0:
            parts.append(
                f'<visual name="d{i}"><pose>{x:.2f} {y:.2f} 0.045 0 0 {h:.4f}</pose>'
                f'<geometry><box><size>1.6 0.18 0.01</size></box></geometry>'
                f'<material><ambient>0.9 0.9 0.9 1</ambient><diffuse>0.9 0.9 0.9 1</diffuse></material></visual>')
    # start/finish line (white) at s=0
    parts.append(
        '<visual name="startline"><pose>4 0 0.06 0 0 0</pose>'
        f'<geometry><box><size>0.4 {road_w:.2f} 0.02</size></box></geometry>'
        '<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material></visual>')
    parts.append('</link></model>')
    # Landmark buildings — well CLEAR of the track (bike never reaches them):
    #   infield centre (x=Ls/2, y=Rc) and far outside both corners.
    blocks = [
        (Ls / 2, Rc, 18, 24, 10, "0.55 0.55 0.6"),         # big infield grandstand (centre)
        (Ls + Rc + 30, Rc, 14, 22, 16, "0.5 0.55 0.6"),    # outside right corner, +30 m clear
        (-Rc - 30, Rc, 14, 22, 16, "0.62 0.6 0.5"),         # outside left corner, +30 m clear
        (Ls / 2, -Rc - 25, 40, 8, 6, "0.6 0.58 0.5"),       # outside front straight
        (Ls / 2, 2 * Rc + Rc + 25, 40, 8, 6, "0.58 0.6 0.55"),  # outside back straight
    ]
    for j, (bx, by, sx, sy, h, c) in enumerate(blocks):
        parts.append(
            f'    <model name="blk{j}"><static>true</static><link name="l">'
            f'<collision name="c"><pose>{bx} {by} {h/2} 0 0 0</pose><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>'
            f'<visual name="v"><pose>{bx} {by} {h/2} 0 0 0</pose><geometry><box><size>{sx} {sy} {h}</size></box></geometry>'
            f'<material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual></link></model>')
    parts.append(_WORLD_FOOTER)
    return "".join(parts)
