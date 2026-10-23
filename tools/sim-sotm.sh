#!/bin/sh
# Shoot-on-the-move test in the simulator: scripted runs (still, strafing, toward and away from the
# hub, an arc, turning) while the robot holds the shoot button; prints "SOTM ..." lines with balls
# shot, balls scored and the mean miss from the hub center for each run.
#
#   sh tools/sim-sotm.sh                       # every run
#   SIM_SOTM_ONLY="strafe" sh tools/sim-sotm.sh # only runs whose name has "strafe"
#   SIM_SOTM_COMPARE=1 sh tools/sim-sotm.sh     # each run again with the lead off
#   SIM_SOTM_TRACE=1 sh tools/sim-sotm.sh       # also the shot values every 0.1 s
#
# Needs JAVA_HOME set to the WPILib JDK. Uses the NetworkTables port, so it stops a sim of this
# project that is still running.
# The robot code runs with no calibration file (constants only), whatever a GUI sim saved.
set -u
cd "$(dirname "$0")/.."
LIB="$PWD/build/jni/release"
LOG="$PWD/build/sim-sotm.log"
rm -f "$LOG" build/no-calibration.json
OLD=$(pgrep -f "bin/java .*$PWD/build/classes/java/main.* frc.robot.Main" || true)
if [ -n "$OLD" ]; then
    echo "Stopping a sim of this project left from an earlier run (PID $OLD)"
    kill $OLD
    sleep 2
fi
sh ./gradlew build -q -x test || { echo "FAIL: build"; exit 1; }
CALIBRATION_FILE="$PWD/build/no-calibration.json" SIM_SOTM=1 SIM_SOTM_ONLY="${SIM_SOTM_ONLY:-}" SIM_SOTM_COMPARE="${SIM_SOTM_COMPARE:-0}" SIM_SOTM_TRACE="${SIM_SOTM_TRACE:-0}" DYLD_LIBRARY_PATH="$LIB" LD_LIBRARY_PATH="$LIB" JAVA_TOOL_OPTIONS="-Djava.library.path=$LIB" \
    sh ./gradlew run -q > "$LOG" 2>&1
grep "SOTM" "$LOG"
if grep -nE "Exception|at frc\.robot\." "$LOG" > /dev/null; then
    echo "FAIL: exceptions in $LOG:"
    grep -nE "Exception|at frc\.robot\." "$LOG" | head -20
    exit 1
fi
grep -q "SOTM end" "$LOG" || { echo "FAIL: the run did not finish (see $LOG)"; exit 1; }
echo "PASS"
