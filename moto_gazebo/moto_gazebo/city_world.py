"""city_world.py — procedural city/road world SDF for Gazebo Classic 11.

Generates a grid of roads + intersections lined with buildings of varying height.
The result is a static world the sensor_bike rides through; its camera/lidar see
the buildings (good landmarks for SLAM). Deterministic given a seed.

Frame: x forward, y left, z up (REP-103).
"""

from __future__ import annotations

import random


_HEADER = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="city">
    <physics type="ode"><max_step_size>0.004</max_step_size><real_time_update_rate>250</real_time_update_rate></physics>
    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so"><ros><namespace>/moto</namespace></ros></plugin>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/moto</namespace></ros><update_rate>100</update_rate></plugin>
    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.7 0.8 0.9 1</background><shadows>true</shadows></scene>
    <include><uri>model://sun</uri></include>
"""

_GROUND = """    <model name="ground"><static>true</static>
      <link name="link"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>
          <material><ambient>0.25 0.45 0.2 1</ambient><diffuse>0.25 0.45 0.2 1</diffuse></material></visual>
      </link></model>
"""

_FOOTER = "  </world>\n</sdf>\n"


def _road(name, x, y, length, width, yaw):
    return f"""    <model name="{name}"><static>true</static>
      <link name="l"><pose>{x:.2f} {y:.2f} 0.01 0 0 {yaw:.4f}</pose>
        <visual name="v"><geometry><box><size>{length:.2f} {width:.2f} 0.02</size></box></geometry>
          <material><ambient>0.18 0.18 0.2 1</ambient><diffuse>0.2 0.2 0.22 1</diffuse></material></visual>
        <collision name="c"><geometry><box><size>{length:.2f} {width:.2f} 0.02</size></box></geometry></collision>
      </link></model>
"""


def _building(name, x, y, sx, sy, h, color):
    r, g, b = color
    return f"""    <model name="{name}"><static>true</static>
      <link name="l"><pose>{x:.2f} {y:.2f} {h/2:.2f} 0 0 0</pose>
        <collision name="c"><geometry><box><size>{sx:.2f} {sy:.2f} {h:.2f}</size></box></geometry></collision>
        <visual name="v"><geometry><box><size>{sx:.2f} {sy:.2f} {h:.2f}</size></box></geometry>
          <material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse></material></visual>
      </link></model>
"""


def generate_city(blocks: int = 4, block_size: float = 40.0, road_width: float = 8.0,
                  seed: int = 7) -> str:
    """A blocks×blocks grid. Roads along grid lines; buildings fill the blocks."""
    rng = random.Random(seed)
    parts = [_HEADER, _GROUND]
    span = blocks * block_size
    half = span / 2.0

    # roads along x (horizontal) and y (vertical) grid lines
    for i in range(blocks + 1):
        c = -half + i * block_size
        parts.append(_road(f"road_h_{i}", 0.0, c, span + road_width, road_width, 0.0))
        parts.append(_road(f"road_v_{i}", c, 0.0, road_width, span + road_width, 0.0))

    # buildings: one cluster per block interior
    palette = [(0.6, 0.6, 0.65), (0.55, 0.5, 0.45), (0.5, 0.55, 0.6), (0.65, 0.6, 0.5)]
    bidx = 0
    for ix in range(blocks):
        for iy in range(blocks):
            cx = -half + (ix + 0.5) * block_size
            cy = -half + (iy + 0.5) * block_size
            # 1–2 buildings per block
            for _ in range(rng.randint(1, 2)):
                sx = rng.uniform(8, block_size - road_width - 6)
                sy = rng.uniform(8, block_size - road_width - 6)
                h = rng.uniform(6, 35)
                ox = cx + rng.uniform(-6, 6)
                oy = cy + rng.uniform(-6, 6)
                parts.append(_building(f"bldg_{bidx}", ox, oy, sx, sy, h, rng.choice(palette)))
                bidx += 1

    parts.append(_FOOTER)
    return "".join(parts)


# ── Endless streaming support ────────────────────────────────────────────────
# An (almost) empty world = giant ground + lights + plugins. The city_streamer
# node spawns/deletes block tiles around the bike at runtime for an infinite city.

def generate_empty_world() -> str:
    big_ground = """    <model name="ground"><static>true</static>
      <link name="link"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>100000 100000</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>100000 100000</size></plane></geometry>
          <material><ambient>0.25 0.45 0.2 1</ambient><diffuse>0.25 0.45 0.2 1</diffuse></material></visual>
      </link></model>
"""
    return _HEADER.format(name="endless_city", mu=1.1) + big_ground + _FOOTER


# Tunables shared with the streamer + roam controller (the road grid geometry).
TILE_SIZE = 140.0     # m — large blocks so the bike's gentle ~76 m-radius turns fit
                      # (the simplified model only corners cleanly at wide radius)
ROAD_WIDTH = 10.0     # wider roads → forgiving corridor for the kinematic bike
SETBACK = 5.0         # m gap road→building; > bike turn-arc corner-cut (~8 m radius)


def tile_model_sdf(i: int, j: int, tile_size: float = TILE_SIZE,
                   road_width: float = ROAD_WIDTH) -> str:
    """One city block tile at grid cell (i,j), absolute world coords. Deterministic.
    Roads run on the tile's min-x and min-y edges (so adjacent tiles align into a
    continuous grid). Buildings are placed STRICTLY inside the clear interior box so
    they never intrude on a road corridor — the bike (kinematic, no collision) stays
    clear as long as it's on/near a road."""
    rng = random.Random((i * 73856093) ^ (j * 19349663))
    x0, y0 = i * tile_size, j * tile_size
    cx, cy = x0 + tile_size / 2, y0 + tile_size / 2
    palette = [(0.6, 0.6, 0.65), (0.55, 0.5, 0.45), (0.5, 0.55, 0.6), (0.65, 0.6, 0.5)]
    parts = [f'<model name="tile_{i}_{j}"><static>true</static><link name="l">']

    def vbox(x, y, z, sx, sy, sz, rgb):
        r, g, b = rgb
        return (f'<collision name="c{len(parts)}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>'
                f'<geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry></collision>'
                f'<visual name="v{len(parts)}"><pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>'
                f'<geometry><box><size>{sx:.2f} {sy:.2f} {sz:.2f}</size></box></geometry>'
                f'<material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>'
                f'<diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse></material></visual>')

    road = (0.18, 0.18, 0.2)
    parts.append(vbox(cx, y0, 0.01, tile_size, road_width, 0.02, road))   # road along min-y edge
    parts.append(vbox(x0, cy, 0.01, road_width, tile_size, 0.02, road))   # road along min-x edge

    # Clear interior box in ABSOLUTE coords. Roads (incl. neighbours' on the max edges)
    # occupy road_width/2 in from each edge → buildings must stay inside [lo, hi].
    half = road_width / 2 + SETBACK
    lo_x, hi_x = x0 + half, x0 + tile_size - half
    lo_y, hi_y = y0 + half, y0 + tile_size - half
    span = min(hi_x - lo_x, hi_y - lo_y)
    if span > 6:
        for _ in range(rng.randint(1, 2)):
            fp_x = rng.uniform(7, max(7, span))
            fp_y = rng.uniform(7, max(7, span))
            bxc = rng.uniform(lo_x + fp_x / 2, hi_x - fp_x / 2)
            byc = rng.uniform(lo_y + fp_y / 2, hi_y - fp_y / 2)
            h = rng.uniform(6, 34)
            parts.append(vbox(bxc, byc, h / 2, fp_x, fp_y, h, rng.choice(palette)))

    parts.append('</link></model>')
    return f'<?xml version="1.0" ?><sdf version="1.6">{"".join(parts)}</sdf>'


# A simple straight "road test" world for first bring-up (fewer parts to load).
def generate_straight_road(length: float = 200.0, width: float = 10.0, n_buildings: int = 20,
                           seed: int = 3) -> str:
    rng = random.Random(seed)
    parts = [_HEADER, _GROUND, _road("main_road", 0, 0, length, width, 0.0)]
    palette = [(0.6, 0.6, 0.65), (0.55, 0.5, 0.45), (0.5, 0.55, 0.6)]
    for i in range(n_buildings):
        x = -length / 2 + (i / max(n_buildings - 1, 1)) * length
        side = 1 if i % 2 == 0 else -1
        y = side * (width / 2 + rng.uniform(6, 12))
        sx, sy, h = rng.uniform(6, 14), rng.uniform(6, 14), rng.uniform(5, 28)
        parts.append(_building(f"b_{i}", x, y, sx, sy, h, rng.choice(palette)))
    parts.append(_FOOTER)
    return "".join(parts)
