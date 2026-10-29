TunerConstants.java:
- Lines 35-36: These can't be right. We never tuned the drive motors and really don't have much time to do that now. Can we just try the suggested numbers that came from CTRE? You need to fix this before you attempt your first auto test.

ShooterSubsystem.java / HoodSubsystem.java:
- You were using the staticShoot to tune the shooter. Are you going to have a staticShoot option during the competition. If so you should probably not still read the numbers from the dashboard.

IntakeExtensionSubsystem.java:
- Line 99: `retractIntake()` has no subsystem requirement (`Commands.run` without `this`), so it can run at the same time as an extend command. Note that of you simply use `run(...)` instead of `Commands.run(...)` you wouldn't need the `this`.
- Lines 139-143: both `retractIntakePower()` and `extendIntakePower()` have issues I can list but they don't seem to be used. However you might want to comment them out now so you don't accidentally use them without fixing.

RobotContainer.java:
- Line 119: Umm, what is this for?
- Line 115: Button 6 turns the turret shutoff on, but nothing turns it back off except restarting the code. Intended?
- Line 51: Is the `whileFalse` needed considering that `returnHood()` is the default command?
