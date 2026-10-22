// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import frc.robot.utils.FuelSim;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.kinematics.ChassisVelocities;

/** Add your docs here. */
public class FuelLaunchSim {
    private Supplier<Pose2d> poseSupplier;
    private Supplier<ChassisVelocities> fieldSpeedsSupplier;
    private int fuelStored = 8;

    public FuelLaunchSim(Supplier<Pose2d> poseSupplier, Supplier<ChassisVelocities> fieldSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    }

    private Translation3d launchVel(double fuelVel, double hoodAngle) {
        Pose2d robot = poseSupplier.get();
        ChassisVelocities fieldSpeeds = fieldSpeedsSupplier.get();

        double horizontalVel = Math.cos(Math.toRadians(hoodAngle)) * fuelVel;
        double verticalVel = Math.sin(Math.toRadians(hoodAngle)) * fuelVel;
        double xVel = horizontalVel * Math.cos(robot.getRotation().getRadians());
        double yVel = horizontalVel * Math.sin(robot.getRotation().getRadians());

        xVel += fieldSpeeds.vx;
        yVel += fieldSpeeds.vy;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    public boolean canIntake() {
        return true;
    }

    public void intakeFuel() {
        fuelStored++;
    }

    public double getFuelStored(){
        return fuelStored;
    }

    public void launchFuel(double vel, double angle) {
        Transform2d leftShotTransform = new Transform2d(0.213, 0.08, new Rotation2d());
        Pose2d shotLeft = poseSupplier.get().transformBy(leftShotTransform);

        Translation3d initialPosLeft = new Translation3d(shotLeft.getX(), shotLeft.getY(), 0.5);

        FuelSim.getInstance().spawnFuel(initialPosLeft, launchVel(vel, angle));


        Transform2d rightShotTransform = new Transform2d(0.213, -0.08, new Rotation2d());
        Pose2d shotRight = poseSupplier.get().transformBy(rightShotTransform);

        Translation3d initialPosRight = new Translation3d(shotRight.getX(), shotRight.getY(), 0.5);

        FuelSim.getInstance().spawnFuel(initialPosRight, launchVel(vel, angle));
    }

    public Command repeatedlyLaunchFuel(
            DoubleSupplier velSupplier, DoubleSupplier angleSupplier) {
        return Commands.runOnce(() -> launchFuel(velSupplier.getAsDouble(), angleSupplier.getAsDouble()))
                .andThen(Commands.waitSeconds(0.2))
                .repeatedly();
    }
}