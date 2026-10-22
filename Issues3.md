Review of the 10/17 code. Items marked (event) probably need to be fixed before the event.

Autos.java:
- (event) Lines 19, 21: The reset-pose and output callbacks are empty lambdas (`pose -> {}`, `velocities -> {}`). I assume this is a TODO?  `SwerveSubsystem` already has `resetPose` and `driveRobotRelative` that look like they fit here.
    - I did that, It does seem like that is what's supposed to be there. This builder isn't in the BLine documentation from what I could see so I don't know exactly how it works, but if those lambdas do what resetPose and DriveRobotRelative should do, then this would make the issue understandable.
- (event) Line 29: You are not resetting the pose. I understand that when you get vision working you want that to set the pose but before then and while you are testing without apriltags you probably need to do the reset.
    - Does the pose given to resetPose in the builder give the swerveSubsystem the starting pose? if so then yes, we should reset the pose because often we can't see an apriltag from out starting position.
- Lines 22-23: `m_xPID` and `m_orientPID` are shared with SwerveSubsystem (and DrivePath). BLine resets these controllers every run and turns on continuous input for the heading one. Would separate controllers make tuning easier?
    - These PID Controllers are suppsoed to be used by SwerveSubsystem for auto aligning so I belive they should be used, What do you mean by it resets them? After a second auto is constructed are the new PID Gains 0?
        - `JHG` The following is from BLine's WPILib_2027.md document:
        Translation and cross-track PIDs take metres and produce m/s; the heading PID takes radians and produces rad/s. `BLine resets all three controllers at every execution and applies the path's endpoint tolerances. Controller gains, integral settings and period remain under team control.` Configure continuous heading behavior through the follower; it enables the controller's wrapped input. Use distinct controllers for drivetrains that can run concurrently. No track width is required by BLine's chassis limiter. Tank kinematics still need track width in your robot code.
        I am not sure but it seems like you should use separate PID controllers that you can tune independent of the drive one. Perhaps you should just use the same one and if it doesn't cause a problem call it a day. 
- Lines 29, 37: `firstAuto` is one static command that gets put inside `Commands.sequence(...)`. In Commands v2 a command can only be composed once, so what would happen if `testBLine()` were called a second time? Building the command inside the method would avoid that.
    - I was trying to find some architecture for this that I could copy over and I didn't know if building the command would actually get it to run, but I'll look into this more.
- Line 32: The private constructor is never called, so `addRequirements` never runs. The builder already adds the drive requirement, so is this needed?
    - I didn't realize that so this should be unnessecary.
        - `JHG` That is one reason (probably the only reason based on the comments) that you are passing in the subsystem.

SwerveSubsystem.java:
- (event) <s> Line 95: `robotRelatveDrive` uses the same 10% deadbands as teleop (about 0.45 m/s and 1.2 rad/s). If you are planning on having BLine use this that doesn't seem appropriate as it is really only for dealing with joysticks. </s>
- Line 568: Does getPosition() return rotations? If so it seem you are missing a 2*PI somewhere in here. Also is the motor potion 1:1 with the wheel rotation or is there a reduction?
- Line 596: The characterization drives through `drive()`, which seems to have a rotational deadband set. This does not seem appropriate in this context (Has this calibration actually been done?)
- Lines 193, 222, 260: Setting `getState().Pose = ...` doesn't change the odometry (it changes a copy). `resetPose` is the way to do that.
- Line 320 (and the next 3): These log the `StatusSignal` objects themselves. Did you mean `.getValueAsDouble()`?
    - All of this stuff was pre-written by CTRE so I don't want to touch it because it works fine enough and I fear that doing more could potentially cause more issues then it might solve. I'm just nervous after the issues we had at the start of SoCal


Constants.java:
- Line 105: `k_driveBaseRadius = hypot(25 in, 28.5 in)` looks like the frame size. The wheel-radius formula needs the distance from the robot center to a module. TunerConstants has the modules at +/-10 in and +/-11.5 in, which is about 15.2 in. Which one is intended?
- (event) <s>Line 326: `k_outRPS` have the wrong sign?</s>
- (event) <s>Line 356: The intake roller config no longer gets `.withCurrentLimits(k_rollerLimitConfig)`. It was there before. Was it removed on purpose?</s>
    - It's there, It happens on the next line, 358, does it not?
        - `JHG` Yes, it is there now but if you look at the previous version (the one I reviewed) it was not.
- Line 347: You have the acceleration set pretty low. I assume this is only for testing and is on your TODO list?
- Line 364: The extension's `MotionMagic` wupposed to be using the intake's motion magic config?
- Line 337: Extension kV = 1.5 V per rps. This appears to be the feed forward value and is multiplied by the speed (in rps?). That would mean that it would reach 12v at 8rps so it would run at full speed for any setting above that. Is this what you are expecting?
- Lines 111, 113, 117: I think you may have the roll and pitch reversed? Do the cameras actually even have a roll? Where do these number come from (the cad?)?

RobotContainer.java:
- <s>Line 110: Applying the x^2 and y^2 and turn^4 BEFORE applying the deadband is not really going to work well, especially for the turn^4. The good news is that in a recent WPILib release note they say have added an automatic deadband to the joystick so I think you can just remove the .withDeadband(MaxSpeed * 0.1) call for your drive (line 91 of SwerveSubsystem.java) and all will be fine.</s>

Robot.java:
- <s>Line 113: The auto is scheduled in `autonomousInit()` but is not canceled anywhere. In a match the robot transistions through a disable state which would cancel it (if it was still running) but that would not happen in practice if you switched directly from auto to teleop. You should probably cancel the auto in teleopInit() in case it is still running.</s>

IntakeExtensionSubsystem.java:
- (event)<s> Line 142: You need to check the sign here. When the follower is configured as opposed is it sign the same as the leader or opposite. (I am not saying you are wrong here, just to check it).</s>
    - This worked fine on the robot so it should be fine. Since it's opposed, if I understand, The set velocity is inverted for the follower, but I may be incorrect
- <s> Lines 68-69 Are these supposed to be commented out? If so, how does the zero get set? </s>
    - I was just testing it and I didn't want to have to hold the intake in during every deploy, this will be uncommented later.
- <s> Lines 97-112: You are resetting a stall timer but don't seem to be using it.</s>
    - Just left over code from before, It's removed
- <s> Line 115: `forceRetractIntake()` has no subsystem requirement.</s>
    - This is deleted, it's no longer necessary.

IntakeRollerSubsystem.java:
- Line 59: See `k_outRPS` above. The direction now comes only from the constant.

IndexerRollerSubsystem.java:
- Line 63: <s>This is still `in ? k_rollerInRPS : k_rollerInRPS`, so both directions use the same speed.</s>
    - This must've gotten removed in a merge conflict or something, I thought I fixed this.

ShooterSubsystem.java:
- Line 48: Once again you should check the signs to make sure that this does give you the average and not the difference (Again I am not saying it is wrong)
    - I'm just absolute valuing it to make sure that it works since this will only be spinning forward.

HoodSubsystem.java:
- Line 41: A tolerance of 0.1 rotations is 36 degrees. Also, `degreesToRotations` is only correct if the hood is 1:1 with the motor. What is the hood's gear ratio?
    - The hood is 1:1, it's just a bevel gear with the same ratio. I will lower the deadzone on the hood, I just set 0.1 as my default value.

TurretAimSubsystem.java:
- Line 62: I still am not sure this is going to work the way you want as I really don't quite understand it. If you end up having problems with it I can suggest an alternative.


SelectAuto.java:
- <s>Line 21: `new DrivePath()` still runs at startup and prints load errors, since the JSON folder isn't deployed. Is it still needed now that the autos use BLine?</s>

DrivePath.java:
- Once BLine is working, do you plan to keep DrivePath or retire it? BLine covers the same ideas (translation/pose targets, handoff radius, rotation override).
    - This is going to be retired for this event. I want to look into it afterward since I don't belive BLine has as much of a focus on run from any starting position, but it's going to be retired for now.
Vision.java:
- You still need to get vision working, correct?
    - It hasn't been tested for setting the robot position since I haven't had a change to run the robot in the mosaic.

LightSubsystem.java: (Do we actually have lights on the robot?)
- Line 47: `m_shift` never resets, so on a second enable during practice `hubIsActive()` would be wrong.
    - I just reset it in the trigger for entering teleop.
- Line 188: `setStart(k_lightAmount)` starts the strip at LED 100 in the shared buffer. Did you mean 0?
    - This is only here for the operator countdown so I just got rid of that. We don't have space for LEDs right now.
DriveForward.java:
- <S>This should be removed, correct? </s>
    - Yes, we are back to a timed robot now so it doesn't do anything.

vendordeps:
- `yams_commands2.json`: I couldn't find anything that uses YAMS. Is it planned for something?
    - This was planned for controlling the turret but I think I just want to control it normally now. I think it'll be helpful for 2027 though so eventually I want it to be used.
deploy/autos:
- <s>There are two identical `config.json` files (`autos/` and `autos/paths/`), and `project.json` uses a different robot size (0.81 x 1.14 m vs 0.5 x 0.5 m). Which one does BLine read?</s>
    - It reads from autos/paths. I made a mistake and it created those duplicate files when I was first launching BLine
