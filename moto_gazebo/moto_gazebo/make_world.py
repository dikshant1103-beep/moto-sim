"""make_world.py — CLI to write a world SDF to disk (used by the launch file).

    python3 -m moto_gazebo.make_world --kind city --out /tmp/city.world
    python3 -m moto_gazebo.make_world --kind straight --out /tmp/road.world
"""

import argparse

from moto_gazebo.city_world import generate_city, generate_straight_road


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kind", choices=["city", "straight"], default="city")
    ap.add_argument("--out", required=True)
    ap.add_argument("--blocks", type=int, default=4)
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    sdf = generate_city(blocks=a.blocks, seed=a.seed) if a.kind == "city" \
        else generate_straight_road(seed=a.seed)
    with open(a.out, "w") as f:
        f.write(sdf)
    print(f"[make_world] wrote {a.kind} → {a.out} ({len(sdf)} bytes)")


if __name__ == "__main__":
    main()
