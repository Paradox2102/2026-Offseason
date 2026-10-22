#!/bin/sh
# Headless simulator check: builds, runs the scripted SimSmokeTest (teleop, BLine auto, vision
# pose kick) with no driver station, and fails if the robot code printed an exception.
#
#   JAVA_HOME=~/wpilib/2027_alpha7/jdk sh tools/sim-smoke.sh
#
# Output: build/sim-smoke.log. The "SMOKE ..." lines give the pose error after each step.
set -u
cd "$(dirname "$0")/.."
LIB="$PWD/build/jni/release"
LOG="$PWD/build/sim-smoke.log"
rm -f "$LOG"
sh ./gradlew build -q || { echo "FAIL: build"; exit 1; }
SIM_SMOKE=1 DYLD_LIBRARY_PATH="$LIB" LD_LIBRARY_PATH="$LIB" JAVA_TOOL_OPTIONS="-Djava.library.path=$LIB" \
    sh ./gradlew run -q > "$LOG" 2>&1
grep "SMOKE" "$LOG"
if grep -nE "Exception|Error:|at frc\.robot\." "$LOG" | grep -v "JAVA_TOOL_OPTIONS" > /dev/null; then
    echo "FAIL: exceptions in $LOG:"
    grep -nE "Exception|Error:|at frc\.robot\." "$LOG" | head -20
    exit 1
fi
grep -q "SMOKE end:" "$LOG" || { echo "FAIL: the run did not finish (see $LOG)"; exit 1; }
echo "PASS"
