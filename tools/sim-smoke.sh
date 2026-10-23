#!/bin/sh
# Headless simulator check: builds, runs the scripted SimSmokeTest (teleop, BLine auto, vision
# pose kick) with no driver station, and fails if the robot code printed an exception or vision did
# not correct the kick (pose and heading).
#
#   JAVA_HOME=~/wpilib/2027_alpha7/jdk sh tools/sim-smoke.sh
#
# Output: build/sim-smoke.log. The "SMOKE ..." lines give the pose error after each step.
# To watch it in AdvantageScope: File > Connect to Simulator (or the sim button) before you start,
# and set SIM_SMOKE_STAY=120 to keep the sim up 2 minutes after the script. The robot pose is the
# root topic "RobotPose", the moving parts "Final Poses"; the sim's true pose is
# /Telemetry/Sim/Truth Pose and the vision poses /Telemetry/Vision/... The run also writes a
# .wpilog in logs/ that AdvantageScope can open afterwards.
# Fuel and the bump: in AdvantageScope, File > Import Layout > tools/advantagescope-sim-layout.json.
# Its "Sim 3D" tab shows the true robot ("RobotPose3d", it climbs the bumps and stops at the field
# structures), the odometry estimate as a green ghost ("RobotPose"), and the balls ("Sim/Fuel" as
# Fuel game pieces; the field's own staged balls hide once a game piece source is there). The fuel
# resets at auto start (Elastic Tuning tab: "Sim: Reset Fuel").
# The "Trench guard" tab graphs the intake position (left axis), the hood angle (right axis) and
# "Near Trench" (stripes): watch the teleop trench run at the end of the script.
# SIM_SMOKE_ALLIANCE=red runs the auto as red (the auto checks are then given in the blue frame).
# SIM_SMOKE_AUTO=right|left|right-sotm|left-sotm picks the trench auto (sotm: the double trench run that
# shoots on the move); a left auto's checks are mirrored onto the right side.
# SIM_SMOKE_RACK=blocked|short|dead: a rack fault during the auto (left side blocked at 0.10 m, right
# side held 2.4 cm short, or Talon 22 dead): it must take the fallback path, not the trench.
# The robot code runs with no calibration file (constants only), whatever a GUI sim saved.
set -u
cd "$(dirname "$0")/.."
LIB="$PWD/build/jni/release"
LOG="$PWD/build/sim-smoke.log"
rm -f "$LOG" build/no-calibration.json build/sim-topics.txt
# A sim of this project left from an earlier run holds the NetworkTables port (5810): the new sim
# could not serve AdvantageScope, which would keep showing the old one. Stop it first.
OLD=$(pgrep -f "bin/java .*$PWD/build/classes/java/main.* frc.robot.Main" || true)
if [ -n "$OLD" ]; then
    echo "Stopping a sim of this project left from an earlier run (PID $OLD)"
    kill $OLD
    sleep 2
fi
sh ./gradlew build -q || { echo "FAIL: build"; exit 1; }
CALIBRATION_FILE="$PWD/build/no-calibration.json" SIM_SMOKE=1 SIM_SMOKE_STAY="${SIM_SMOKE_STAY:-0}" SIM_SMOKE_ALLIANCE="${SIM_SMOKE_ALLIANCE:-blue}" SIM_SMOKE_AUTO="${SIM_SMOKE_AUTO:-right}" SIM_SMOKE_RACK="${SIM_SMOKE_RACK:-}" SIM_SMOKE_TRACE="${SIM_SMOKE_TRACE:-0}" DYLD_LIBRARY_PATH="$LIB" LD_LIBRARY_PATH="$LIB" JAVA_TOOL_OPTIONS="-Djava.library.path=$LIB" \
    sh ./gradlew run -q > "$LOG" 2>&1
grep "SMOKE" "$LOG"
if grep -nE "Exception|Error:|at frc\.robot\." "$LOG" | grep -v "JAVA_TOOL_OPTIONS" > /dev/null; then
    echo "FAIL: exceptions in $LOG:"
    grep -nE "Exception|Error:|at frc\.robot\." "$LOG" | head -20
    exit 1
fi
grep -q "SMOKE end:" "$LOG" || { echo "FAIL: the run did not finish (see $LOG)"; exit 1; }
python3 tools/check-elastic-layout.py src/main/deploy/elastic-layout.json build/sim-topics.txt || { echo "FAIL: layout"; exit 1; }
if grep -q "SMOKE FAIL" "$LOG"; then grep "SMOKE FAIL" "$LOG"; exit 1; fi
echo "PASS"
