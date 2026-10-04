package frc.robot.utils;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.ConditionalCommand;

import frc.robot.Constants.Controls;
import frc.robot.Constants.Subsystems;
import frc.robot.PathPlanningThing.DrivePath;

public class Autos {
    public static Command doubleSwipe(boolean right){
        return Commands.parallel(
            Commands.repeatingSequence(Commands.either(Subsystems.m_intakeRoller.run(true), Commands.none(), () -> !Subsystems.m_intakeExtension.isIntakeRetracted())),
            Commands.sequence(
            Subsystems.m_intakeExtension.extendIntake(),
            new DrivePath("RightDoubleSwipe1.json", !right),
            
            Commands.parallel(
                new DrivePath("RightDoubleSwipe2.json", !right),
            Controls.shoot()
            ),
            Commands.parallel(
                new ConditionalCommand(Subsystems.m_intakeExtension.extendIntake(), Commands.none(), () -> Subsystems.m_intakeExtension.isIntakeRetracted()),
                new DrivePath("RightDoubleSwipe3.json", !right)
            ),
            Commands.parallel(
                new DrivePath("RightDoubleSwipe4.json", !right),
            Controls.shoot()
            )
        ));
    }
}
