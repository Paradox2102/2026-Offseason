Constants.java:
- <s>Line 406: shoot() never actually feeds. Commands.either picks a branch once, when it starts. Neither launch() nor aim() ever finishes, so repeatingSequence never repeats. If the robot isn’t ready at the exact moment the button is pressed, it aims forever and never shoots. A fix is to run aiming all the time and gate only the feeders.</s>
- <s>Line 389: Shouldn't this be using k_indexerConveyorConfigs? </s>

ShooterSubsystem.java:
- <s>Line 49: m_isAtSpeed() is always true. The abs only wraps the average, not the difference from the target, so any speed below target + 3 counts as “at speed”, including 0. There’s a second problem: the opposed follower reads roughly the negative of the leader’s velocity, so the average is about 0 anyway.</s>

TurretAimSubsystem.java:
- <s>Line 42: m_isOnTarget() will always true. It compares the requested position to the target, which are the same number. It needs to compare the motor’s measured position to the target.</s>
- Line 62: getTargetRotation() has a number of issues. I can detail them or I can suggest a replacement. I strongly recommend that you do NOT attempt shoot on the fly until you get the static shoot working.    
    - The shoot on the fly code is entirely in swerve subsystem, taken from Helios. The actual turret aiming is only doing weird math to account for the turret not being where the robot's center of rotaton is. I will look at solutions by teams like 4322 with open turret code to see what they do.

IntakeExtensionSubsystem.java:
- Line 118: I am pretty sure isIntakeAtTarget() is not doing what you think it is.
    - It's checking if the average of the two motor's position - the target is less than a value to see if we're within a deadzone correct? I dead realize that I subtracted in my average, but that's now fixed

HoodSubsystem.java:
- <s> Line 80: staticPitch() - k_staticShotAngle seems to be degrees but isn't setRequest expecting rotations? </s>

IntakeRollerSubsystem:
- Line 51: I believe the BangBang only ever return 0 or 1 so it will not reverse. In any case I am not sure that using BangBang is the right choice here.
    - I fixed the reverse issue, but the reason I decided a bang-bang would be useful here is because when we're intaking, we don't care about our speed, just that we stay spinning fast enough to actually intake as our robot drives around fast. That means that we can overshoot, but when we're sweeping through a hoard of fuel in the beginning of autonomous our intake won't drop down too slow that we push as opposed to intake.

IndexerConveyorSubsystem:
- <s> Line 70: Since you are using '<' you should not be subtracting 1 here. </s>
    - I get confused when some languages want me to do a minus one when getting a size of a list or when some don't want me to.

DrivePath.java
- Lines 74,112,162: Are you sure that rotations are in the correct units (degrees vs radians)?
- Line 118: I don't think this will do anything. yPosition is used above to calculate m_translation and m_pose but is not used after this loop so it doesn't seem like it will have any effect. Seems like the mirror code should be moved up.
- Line 257: This loop will never run. Should it be i < m_pose.length?
- Line 266: This loop will also never run since i is never negative. And it is a good thing too since you would be indexing an array with a negative.
- You have a lot of println that print every 20ms. I assume this is for debugging but you are going to get a lot of output.
    - This isn't going to be used during beach blitz so, for the sake of time, I'm not going to resolve these issues just yet

DriveForward.java:
This is done completely incorrectly. I know you had a member of the other team writing this but he clearly didn't know how commands work. This should be a command, not directly calling setControl. While this is running the default teleop command will also be running which will also be trying to setControl. Since you will eventually be using PathPlanner this doesn't need to be fixed, just removed. However do ever call the setControl outside a command.
- Right now it's staying in just so I have an example for the syntax of opmode, It was written quite hastily and will be removed, but I'll keep this in mind for later, but If I remember how he explained it, the whole opmode is a command with an initialize and periodic, it's just a different syntax. 
