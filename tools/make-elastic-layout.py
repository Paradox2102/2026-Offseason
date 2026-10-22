"""Writes src/main/deploy/elastic-layout.json (Elastic 2027 layout format).

Edit the tab lists below and run:  python3 tools/make-elastic-layout.py src/main/deploy/elastic-layout.json
tools/sim-smoke.sh then checks that every widget topic exists in the simulator.
Grid units: one cell = 128 px. Colors are ARGB integers.
"""
import json, sys
G = 128
GREEN, RED, BLUE, ORANGE = 0xFF4CAF50, 0xFFF44336, 0xFF2196F3, 0xFFFF9800

def w(title, typ, topic, x, y, cw, ch, **props):
    p = {"topic": topic, "period": 0.06}
    p.update(props)
    return {"title": title, "x": x * G, "y": y * G, "width": cw * G, "height": ch * G, "type": typ, "properties": p}

def boolbox(title, topic, x, y, cw=1, ch=1):
    return w(title, "Boolean Box", topic, x, y, cw, ch, true_color=GREEN, false_color=RED, true_icon="None", false_icon="None")

def graph(title, topic, x, y, cw=3, ch=2, color=BLUE, **kw):
    return w(title, "Graph", topic, x, y, cw, ch, time_displayed=10.0, color=color, line_width=2.0, **kw)

def text(title, topic, x, y, cw=2, ch=1):
    return w(title, "Text Display", topic, x, y, cw, ch, show_submit_button=False)

def command(title, topic, x, y, cw=2, ch=1):
    return w(title, "Command", topic, x, y, cw, ch, show_type=False, maximize_button_space=True)

field_props = dict(field_game="Rebuilt", coordinate_system="wall_blue", robot_width=0.81, robot_length=0.90,
                   show_other_objects=True, show_trajectories=True, field_rotation=0.0,
                   robot_color=RED, trajectory_color=0xFFFFFFFF, show_robot_outside_widget=True)

driver = [
    w("Field", "Field", "/Telemetry/Field", 0, 0, 7, 4, **field_props),
    w("Match Time", "Match Time", "/Telemetry/Match Time", 7, 0, 2, 1,
      time_display_mode="Minutes and Seconds", red_start_time=20, yellow_start_time=30),
    boolbox("Vision Target", "/Telemetry/Vision/Has Target", 9, 0),
    w("Battery", "Voltage View", "/Telemetry/Health/Battery V", 10, 0, 2, 1,
      min_value=8.0, max_value=13.0, divisions=5, inverted=False, orientation="horizontal"),
    w("Auto", "Dropdown Chooser", "/Tunables/Auto Selection", 7, 1, 3, 1, sort_options=False),
    w("Alerts", "Alerts", "/Telemetry/Alerts/Health", 7, 2, 5, 2),
]

pit = [
    w("Alerts", "Alerts", "/Telemetry/Alerts/Health", 0, 0, 5, 4),
    command("Swerve Self-Test", "/Tunables/Pit/Swerve Self-Test", 5, 0),
    command("Wheels Forward", "/Tunables/Pit/Wheels Forward", 7, 0),
    command("Reset Heading", "/Tunables/Pit/Reset Heading", 9, 0),
    graph("Battery V", "/Telemetry/Health/Battery V", 5, 1, color=GREEN, min_value=6.0, max_value=13.5),
    graph("Loop Time ms", "/Telemetry/Health/Loop Time ms", 8, 1, color=ORANGE),
    graph("CAN S1 (indexer) %", "/Telemetry/Health/CAN/CAN_S1/Utilization", 5, 3, ch=1, cw=2),
    graph("CAN S2 (swerve) %", "/Telemetry/Health/CAN/CAN_S2/Utilization", 7, 3, ch=1, cw=2),
    graph("CAN S3 (shooter) %", "/Telemetry/Health/CAN/CAN_S3/Utilization", 9, 3, ch=1, cw=2),
    graph("CAN S4 (intake) %", "/Telemetry/Health/CAN/CAN_S4/Utilization", 11, 3, ch=1, cw=2),
]

drive = [
    w("Swerve", "SwerveDrive", "/Telemetry/Swerve Drive", 0, 0, 3, 3, show_robot_rotation=True, rotation_unit="Radians"),
    graph("Speed m/s", "/Telemetry/Drive/Speed mps", 3, 0, cw=4, color=BLUE),
    graph("Odometry Hz", "/Telemetry/Drive/Odometry Hz", 7, 0, cw=3, color=ORANGE),
    text("Heading (gyro) deg", "/Telemetry/Drive/Raw Heading deg", 3, 2),
    text("Failed DAQs", "/Telemetry/Drive/Failed DAQs", 5, 2),
    w("Open loop teleop", "Toggle Switch", "/Tunables/Drive/Open loop teleop", 7, 2, 2, 1),
]

vision = []
for i, cam in enumerate(["front_right", "front_left", "side_left", "side_right"]):
    x = i * 3
    base = f"/Telemetry/Vision/{cam}/"
    vision += [
        boolbox(cam, base + "Connected", x, 0, 3, 1),
        text("Tags", base + "Tag Count", x, 1, 1, 1),
        text("Reject reason", base + "Reject Reason", x + 1, 1, 2, 1),
        graph("Robot roll deg (0 = camera OK)", base + "Robot Roll deg", x, 2, cw=3, ch=1, color=ORANGE),
        graph("Robot pitch deg (0 = camera OK)", base + "Robot Pitch deg", x, 3, cw=3, ch=1, color=ORANGE),
        graph("Robot z m (0 = camera OK)", base + "Robot Z m", x, 4, cw=3, ch=1, color=BLUE),
    ]

tuning = [text("SysId routine (test pad)", "/Telemetry/Drive/SysId Routine", 0, 0, 3, 1)]
for r, name in enumerate(["Translation", "Steer", "Rotation"]):
    for c, test in enumerate(["Quasistatic Forward", "Quasistatic Reverse", "Dynamic Forward", "Dynamic Reverse"]):
        tuning.append(command(f"{name} {test}", f"/Tunables/Tuning/SysId {name}/{test}", c * 3, 1 + r, 3, 1))
tuning += [
    command("Wheel Radius", "/Tunables/Tuning/Wheel Radius", 0, 4, 3, 1),
    command("Sim: Kick Pose", "/Tunables/Sim/Kick Pose", 3, 4, 3, 1),
    w("Open loop teleop", "Toggle Switch", "/Tunables/Drive/Open loop teleop", 6, 4, 2, 1),
]

layout = {"version": 1.0, "grid_size": G, "tabs": [
    {"name": n, "grid_layout": {"layouts": [], "containers": c}}
    for n, c in [("Driver", driver), ("Pit", pit), ("Drive", drive), ("Vision", vision), ("Tuning", tuning)]]}
json.dump(layout, open(sys.argv[1], "w"), indent=2)
print(sum(len(t["grid_layout"]["containers"]) for t in layout["tabs"]), "widgets")
