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
        text("Latency ms", base + "Latency ms", x, 5, 3, 1),
    ]
# Phoenix clock minus the vision (NetworkTables) clock and minus the WPILib clock: steady = OK.
vision += [
    text("Clock offset ms (vision)", "/Telemetry/Vision/Clock Offset ms", 0, 6, 3, 1),
    text("Clock offset ms (timer)", "/Telemetry/Vision/Timer Offset ms", 3, 6, 3, 1),
]

tuning = [text("SysId routine (test pad, Calibration tab)", "/Telemetry/Drive/SysId Routine", 0, 0, 3, 1)]
for r, name in enumerate(["Translation", "Steer", "Rotation"]):
    for c, test in enumerate(["Quasistatic Forward", "Quasistatic Reverse", "Dynamic Forward", "Dynamic Reverse"]):
        tuning.append(command(f"{name} {test}", f"/Tunables/Tuning/SysId {name}/{test}", c * 3, 1 + r, 3, 1))
tuning += [
    command("Wheel Radius", "/Tunables/Tuning/Wheel Radius", 0, 4, 3, 1),
    command("Sim: Kick Pose", "/Tunables/Sim/Kick Pose", 3, 4, 3, 1),
    w("Open loop teleop", "Toggle Switch", "/Tunables/Drive/Open loop teleop", 6, 4, 2, 1),
    # Sim only: the fuel (see sim/FuelSim.java).
    command("Sim: Reset Fuel", "/Tunables/Sim/Reset Fuel", 0, 5, 3, 1),
    text("Sim hopper", "/Telemetry/Sim/Fuel/Hopper", 3, 5, 2, 1),
    text("Sim picked up", "/Telemetry/Sim/Fuel/Picked Up", 5, 5, 2, 1),
    text("Sim scored blue", "/Telemetry/Sim/Fuel/Scored Blue", 7, 5, 2, 1),
    text("Sim scored red", "/Telemetry/Sim/Fuel/Scored Red", 9, 5, 2, 1),
]

intake = [
    boolbox("Rack at stop", "/Telemetry/Intake/Rack/At Stop", 0, 0, 2, 1),
    boolbox("Goal out", "/Telemetry/Intake/Rack/Goal Out", 2, 0, 2, 1),
    graph("Rack position m", "/Telemetry/Intake/Rack/Position m", 4, 0, cw=4, color=BLUE, min_value=-0.02, max_value=0.36),
    graph("Skew m (left - right)", "/Telemetry/Intake/Rack/Skew m", 8, 0, cw=4, color=ORANGE),
]
for i, side in enumerate(["Left", "Right"]):
    base = f"/Telemetry/Intake/Rack/{side}/"
    x = i * 2
    intake += [
        text(f"{side} phase", base + "Phase", x, 1, 2, 1),
        text(f"{side} position m", base + "Position m", x, 2, 2, 1),
        text(f"{side} out stop m", base + "Out Stop m", x, 3, 2, 1),
        boolbox(f"{side} rebooted", base + "Lost", x, 4, 2, 1),
    ]
intake += [
    graph("Roller target rps", "/Telemetry/Intake/Roller/Target rps", 4, 2, cw=4, color=GREEN),
    graph("Roller speed rps", "/Telemetry/Intake/Roller/Speed rps", 8, 2, cw=4, color=BLUE),
    boolbox("Unjamming", "/Telemetry/Intake/Roller/Unjamming", 4, 4, 2, 1),
    text("Recent jams", "/Telemetry/Intake/Roller/Recent Jams", 6, 4, 2, 1),
]
# A rebooted side should show red: swap the colors of those boxes.
for w_ in intake:
    if w_["title"].endswith("rebooted"):
        w_["properties"]["true_color"], w_["properties"]["false_color"] = RED, GREEN

shooter = [
    boolbox("Ready to shoot", "/Telemetry/Shooter/Ready", 0, 0, 2, 1),
    boolbox("Scoring (else pass)", "/Telemetry/Shooter/Scoring", 2, 0, 2, 1),
    text("Distance m", "/Telemetry/Shooter/Distance m", 4, 0, 2, 1),
    text("Feeder", "/Telemetry/Shooter/Feeder/State", 6, 0, 2, 1),
    boolbox("Turret zeroed", "/Telemetry/Shooter/Turret/Zeroed", 8, 0, 2, 1),
    boolbox("Hood homed", "/Telemetry/Shooter/Hood/Homed", 10, 0, 2, 1),
    graph("Turret error deg", "/Telemetry/Shooter/Turret/Error deg", 0, 1, cw=4, color=ORANGE, min_value=-10, max_value=10),
    graph("Turret position deg", "/Telemetry/Shooter/Turret/Position deg", 4, 1, cw=4, color=BLUE),
    graph("Hood deg", "/Telemetry/Shooter/Hood/Position deg", 8, 1, cw=4, color=GREEN),
    graph("Flywheel rps", "/Telemetry/Shooter/Flywheel/Speed rps", 0, 3, cw=6, color=RED),
    graph("Flywheel target rps", "/Telemetry/Shooter/Flywheel/Target rps", 6, 3, cw=6, color=BLUE),
    command("Zero turret (in jig)", "/Tunables/Pit/Zero turret (in jig)", 0, 5, 3, 1),
]
pit.append(command("Zero turret (in jig)", "/Tunables/Pit/Zero turret (in jig)", 11, 0))
driver.append(boolbox("Shot ready", "/Telemetry/Shooter/Ready", 10, 1, 2, 1))
# Trench rule: near a trench the intake goes out and the hood down (Superstructure, TrenchGuard).
driver.append(boolbox("Near trench", "/Telemetry/Shooter/Near Trench", 7, 4, 2, 1))
driver.append(boolbox("Intake fully out", "/Telemetry/Shooter/Intake Extended", 9, 4, 2, 1))
shooter.append(boolbox("Near trench", "/Telemetry/Shooter/Near Trench", 3, 5, 2, 1))
shooter.append(boolbox("Intake fully out", "/Telemetry/Shooter/Intake Extended", 5, 5, 2, 1))

# Calibration (Utility mode only; see calibration/Calibration.java). The test pad works only for the
# procedure picked here.
def entry(title, topic, x, y, cw=3, ch=1):
    return w(title, "Text Display", topic, x, y, cw, ch, show_submit_button=True)

calibration = [
    w("Procedure", "Dropdown Chooser", "/Tunables/Calibration/Procedure", 0, 0, 3, 1, sort_options=False),
    boolbox("Utility mode", "/Telemetry/Calibration/Utility Mode", 3, 0, 2, 1),
    text("Step", "/Telemetry/Calibration/Step", 5, 0, 7, 1),
    text("Instructions", "/Telemetry/Calibration/Instructions", 0, 1, 12, 1),
    text("Reading 1", "/Telemetry/Calibration/Reading 1", 0, 2, 6, 1),
    text("Reading 2", "/Telemetry/Calibration/Reading 2", 6, 2, 6, 1),
    text("Reading 3", "/Telemetry/Calibration/Reading 3", 0, 3, 6, 1),
    text("Reading 4", "/Telemetry/Calibration/Reading 4", 6, 3, 6, 1),
    command("Capture", "/Tunables/Calibration/Capture", 0, 4, 2, 1),
    command("Save", "/Tunables/Calibration/Save", 2, 4, 2, 1),
    text("Message", "/Telemetry/Calibration/Message", 4, 4, 8, 1),
    entry("Shot: typed distance m (0 = pose)", "/Tunables/Calibration/Shot/Typed Distance m", 0, 5),
    entry("Shot: typed time of flight s", "/Tunables/Calibration/Shot/Typed Time of Flight s", 3, 5),
    entry("Hood: upper stop angle deg", "/Tunables/Calibration/Hood/Upper Stop Angle deg", 6, 5),
    entry("Hood: lower stop above level deg", "/Tunables/Calibration/Hood/Lower Stop Above Level deg", 9, 5),
    entry("Pack fraction", "/Tunables/Intake/Pack/Fraction", 0, 6),
    entry("Pulse amplitude m", "/Tunables/Intake/Pack/Pulse Amplitude m", 3, 6),
    entry("Pulse rate Hz", "/Tunables/Intake/Pack/Pulse Rate Hz", 6, 6),
    text("SysId routine", "/Telemetry/Drive/SysId Routine", 9, 6, 3, 1),
]

layout = {"version": 1.0, "grid_size": G, "tabs": [
    {"name": n, "grid_layout": {"layouts": [], "containers": c}}
    for n, c in [("Driver", driver), ("Pit", pit), ("Drive", drive), ("Vision", vision), ("Intake", intake), ("Shooter", shooter), ("Tuning", tuning),
                 ("Calibration", calibration)]]}
json.dump(layout, open(sys.argv[1], "w"), indent=2)
print(sum(len(t["grid_layout"]["containers"]) for t in layout["tabs"]), "widgets")
