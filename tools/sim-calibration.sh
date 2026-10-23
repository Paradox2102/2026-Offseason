#!/bin/sh
# Calibration tool test in the simulator: in Utility mode (the 2027 name of Test mode) it runs each
# calibration procedure through the same methods the test gamepad calls, checks that a recorded shot
# point changes the shot the robot uses, saves the file and reads it back; then boots again with that
# file and checks that every value in it is in use. Prints "CALIB ..." lines.
#
#   JAVA_HOME=~/wpilib/2027_alpha7/jdk sh tools/sim-calibration.sh
#
# Uses its own calibration file (build/sim-calibration.json, deleted first), so a file saved in a
# GUI sim (build/calibration.json) does not change it. Uses the NetworkTables port, so it stops a
# sim of this project that is still running.
set -u
cd "$(dirname "$0")/.."
LIB="$PWD/build/jni/release"
LOG="$PWD/build/sim-calibration.log"
FILE="$PWD/build/sim-calibration.json"
rm -rf "$LOG" "$FILE" build/calibration-backup
OLD=$(pgrep -f "bin/java .*$PWD/build/classes/java/main.* frc.robot.Main" || true)
if [ -n "$OLD" ]; then
    echo "Stopping a sim of this project left from an earlier run (PID $OLD)"
    kill $OLD
    sleep 2
fi
sh ./gradlew build -q -x test || { echo "FAIL: build"; exit 1; }
SIM_CALIB=1 CALIBRATION_FILE="$FILE" DYLD_LIBRARY_PATH="$LIB" LD_LIBRARY_PATH="$LIB" JAVA_TOOL_OPTIONS="-Djava.library.path=$LIB" \
    sh ./gradlew run -q > "$LOG" 2>&1
# Then boot again with the file it saved: every value must be in use from boot on.
SIM_CALIB=1 SIM_CALIB_BOOT=1 CALIBRATION_FILE="$FILE" DYLD_LIBRARY_PATH="$LIB" LD_LIBRARY_PATH="$LIB" JAVA_TOOL_OPTIONS="-Djava.library.path=$LIB" \
    sh ./gradlew run -q >> "$LOG" 2>&1
grep -E "CALIB|Calibration" "$LOG"
# CTRE's "Warning 10002" (configs applied often) comes with a stack trace: the intake zero re-trues
# the rack's position (a config call) at each stop, 7 times in a few seconds. Not an exception.
ERRORS=$(awk '/^Warning 10002/ {skip = 1} /^$/ {skip = 0} !skip' "$LOG" | grep -nE "Exception|at frc\.robot\.")
if [ -n "$ERRORS" ]; then
    echo "FAIL: exceptions in $LOG:"
    echo "$ERRORS" | head -20
    exit 1
fi
[ "$(grep -c "CALIB end" "$LOG")" = 2 ] || { echo "FAIL: a run did not finish (see $LOG)"; exit 1; }
if grep -q "CALIB FAIL" "$LOG"; then
    echo "FAIL: a check failed"
    exit 1
fi
echo "PASS"
