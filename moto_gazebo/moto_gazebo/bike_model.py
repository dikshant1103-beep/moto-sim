"""bike_model.py — namespaced sensor_bike SDF with a recognizable motorcycle visual.

The bike is KINEMATIC (gravity off), teleported each frame by pose_driver from the
PyChrono odometry. The visual is built from SDF primitives (wheels, frame, tank,
seat, fork, bars, engine, rider) so it reads as a motorcycle without any asset
files. To use a real 3D MESH instead, see MESH_READY below — drop in one <uri>.

Sensors: onboard camera + 2D lidar, published under the bike's namespace, e.g.
/moto/onboard_camera/image_raw/compressed and /moto/scan.

Frame: x forward, y left, z up. Wheel centers at model z=0 (≈ ground when the
pose_driver sets base_link z to the wheel radius).
"""

from __future__ import annotations

WHEELBASE = 1.30
WHEEL_R = 0.30


def _box(name, x, y, z, sx, sy, sz, rgb, rpy="0 0 0"):
    r, g, b = rgb
    return f"""      <visual name="{name}">
        <pose>{x} {y} {z} {rpy}</pose>
        <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual>"""


def _cyl(name, x, y, z, radius, length, rgb, rpy="0 0 0"):
    r, g, b = rgb
    return f"""      <visual name="{name}">
        <pose>{x} {y} {z} {rpy}</pose>
        <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual>"""


def _sphere(name, x, y, z, radius, rgb):
    r, g, b = rgb
    return f"""      <visual name="{name}">
        <pose>{x} {y} {z} 0 0 0</pose>
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>
      </visual>"""


def bike_visuals(color=(0.85, 0.15, 0.15)) -> str:
    """Recognizable motorcycle from primitives, facing +x. `color` = body colour."""
    body = color
    dark = (0.08, 0.08, 0.09)     # tyres
    grey = (0.5, 0.5, 0.55)       # rim / engine
    blk = (0.12, 0.12, 0.14)      # seat / bars
    skin = (0.8, 0.6, 0.5)
    wb = WHEELBASE
    parts = [
        # tyres (cylinders, axis along y → rotate 90° about x)
        _cyl("rear_tyre", 0.0, 0, 0.0, WHEEL_R, 0.12, dark, "1.5708 0 0"),
        _cyl("front_tyre", wb, 0, 0.0, WHEEL_R, 0.11, dark, "1.5708 0 0"),
        _cyl("rear_rim", 0.0, 0, 0.0, WHEEL_R * 0.55, 0.13, grey, "1.5708 0 0"),
        _cyl("front_rim", wb, 0, 0.0, WHEEL_R * 0.55, 0.12, grey, "1.5708 0 0"),
        # swingarm + main frame spar
        _box("swingarm", wb * 0.28, 0, 0.18, 0.56, 0.05, 0.05, grey),
        _box("frame_spar", wb * 0.55, 0, 0.52, 0.7, 0.07, 0.07, body),
        # engine block
        _box("engine", wb * 0.45, 0, 0.34, 0.34, 0.34, 0.34, grey),
        # fuel tank
        _box("tank", wb * 0.55, 0, 0.66, 0.46, 0.26, 0.20, body),
        # seat + tail
        _box("seat", wb * 0.20, 0, 0.64, 0.42, 0.26, 0.07, blk),
        _box("tail", wb * 0.02, 0, 0.66, 0.18, 0.22, 0.12, body),
        # fork (tilted by rake ~ -0.42 rad about y) from front wheel up to steering head
        _cyl("fork", wb - 0.02, 0, 0.50, 0.035, 1.0, grey, "0 -0.42 0"),
        # handlebars (wide bar) + risers
        _box("handlebar", wb - 0.18, 0, 1.00, 0.05, 0.52, 0.05, blk),
        # headlight
        _cyl("headlight", wb + 0.05, 0, 0.78, 0.09, 0.06, (0.95, 0.95, 0.8), "0 1.5708 0"),
        # rider — torso, head, arms suggestion
        _box("rider_torso", wb * 0.42, 0, 1.02, 0.26, 0.30, 0.52, (0.15, 0.2, 0.35)),
        _sphere("rider_head", wb * 0.42, 0, 1.36, 0.12, skin),
    ]
    return "\n".join(parts)


# ── MESH_READY ───────────────────────────────────────────────────────────────
# To use a real 3D motorcycle mesh instead of the primitives above, drop the file
# under models/sensor_bike/meshes/motorcycle.dae and replace the call to
# bike_visuals(...) in sensor_bike_sdf() with bike_mesh_visual(). Adjust scale/pose
# so wheels sit at z≈0 and it faces +x.
def bike_mesh_visual(uri="model://sensor_bike/meshes/motorcycle.dae",
                     scale="1 1 1", rpy="0 0 1.5708") -> str:
    return f"""      <visual name="bike_mesh">
        <pose>0 0 0 {rpy}</pose>
        <geometry><mesh><uri>{uri}</uri><scale>{scale}</scale></mesh></geometry>
      </visual>"""


def sensor_bike_sdf(name: str, ns: str, with_camera: bool = True, with_lidar: bool = True,
                    color=(0.85, 0.15, 0.15), use_mesh: bool = False, light: bool = False) -> str:
    """light=True lowers sensor cost for 4 GB GPU / 14 GB RAM: camera 320×240@15 Hz,
    lidar 360 samples. Full: 640×480@30 Hz, 720 samples."""
    visual = bike_mesh_visual() if use_mesh else bike_visuals(color)
    cam_w, cam_h, cam_rate = (320, 240, 15) if light else (640, 480, 30)
    lidar_n, lidar_rate = (360, 12) if light else (720, 20)

    cam = ""
    if with_camera:
        # Mounted at the FRONT of the bike (ahead of the headlight at x≈1.35), above the
        # wheel, angled slightly down — clear of the tank/bars/rider so nothing blocks it.
        cam = f"""
      <sensor name="onboard_camera" type="camera">
        <pose>{WHEELBASE + 0.15:.2f} 0 1.05 0 0.12 0</pose>
        <update_rate>{cam_rate}</update_rate>
        <camera name="onboard">
          <horizontal_fov>1.5</horizontal_fov>
          <image><width>{cam_w}</width><height>{cam_h}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>200</far></clip>
        </camera>
        <plugin name="{name}_cam" filename="libgazebo_ros_camera.so">
          <ros><namespace>/{ns}</namespace></ros>
          <camera_name>onboard_camera</camera_name>
          <frame_name>{name}/camera_link</frame_name>
        </plugin>
      </sensor>"""
    lidar = ""
    if with_lidar:
        lidar = f"""
      <sensor name="lidar" type="ray">
        <pose>0.1 0 1.0 0 0 0</pose>
        <update_rate>{lidar_rate}</update_rate>
        <ray>
          <scan><horizontal><samples>{lidar_n}</samples><resolution>1</resolution>
            <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
          <range><min>0.3</min><max>60.0</max><resolution>0.01</resolution></range>
        </ray>
        <plugin name="{name}_lidar" filename="libgazebo_ros_ray_sensor.so">
          <ros><namespace>/{ns}</namespace><remapping>~/out:=scan</remapping></ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>{name}/lidar_link</frame_name>
        </plugin>
      </sensor>"""

    return f"""<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="{name}">
    <static>false</static>
    <link name="base_link">
      <gravity>false</gravity>
      <inertial><mass>1.0</mass>
        <inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
{visual}{cam}{lidar}
    </link>
  </model>
</sdf>"""


BIKE_COLORS = [
    (0.85, 0.15, 0.15),  # lead — red
    (0.15, 0.45, 0.85),  # blue
    (0.20, 0.70, 0.40),  # green
    (0.80, 0.65, 0.15),  # amber
    (0.60, 0.30, 0.75),  # purple
]
