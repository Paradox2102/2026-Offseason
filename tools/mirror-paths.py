"""Writes the left-trench auto paths from the right-trench ones.

The left (high-y) trench route is the right route mirrored about the field's long center line:
y -> width - y, heading -> -heading. Every field element the route uses (trench, bump, hub side,
fuel) is mirrored about that line too. Edit the right paths, then run:

    python3 tools/mirror-paths.py

The BLine editor (project.json) lists both sets; the left ones are marked "generated".

Width: the AndyMark field the robot plays on (FieldConstants.kFieldWidth, 8.043 m), the same
size BLine's red flip uses.
"""
import json, math, pathlib

WIDTH = 8.043
PATHS = pathlib.Path(__file__).resolve().parent.parent / "src/main/deploy/autos/paths"

for src in sorted(PATHS.glob("right-trench-*.json")):
    path = json.loads(src.read_text())
    for e in path["path_elements"]:
        if e["type"] == "waypoint":
            t = e["translation_target"]
            t["y_meters"] = round(WIDTH - t["y_meters"], 4)
            r = e["rotation_target"]
            r["rotation_radians"] = round(math.remainder(-r["rotation_radians"], 2 * math.pi), 6)
        elif e["type"] == "translation":
            e["y_meters"] = round(WIDTH - e["y_meters"], 4)
        elif e["type"] == "rotation":
            e["rotation_radians"] = round(math.remainder(-e["rotation_radians"], 2 * math.pi), 6)
    dst = src.with_name(src.name.replace("right-", "left-"))
    dst.write_text(json.dumps(path, indent=2) + "\n")
    print(f"{src.name} -> {dst.name}")
