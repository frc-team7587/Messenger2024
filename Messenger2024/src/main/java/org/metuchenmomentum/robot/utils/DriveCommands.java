package org.metuchenmomentum.robot.utils;

import java.util.function.DoubleSupplier;

import org.metuchenmomentum.robot.Constants.IOConstants;
import org.metuchenmomentum.robot.subsystems.drive.SwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveCommands {

    private static PIDController angleController = new PIDController(0.9, 0, 0);

    /* Commands.run(
            () -> { 
                Pose2d speakerPose = new Pose2d(-0.2, (5 + 6.12)/2, new Rotation2d(0));
                // Apply deadband
                double linearMagnitude =
                    MathUtil.applyDeadband(
                        Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
                Rotation2d linearDirection =
                    new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
                Transform2d targetTransform = drive.getPose().minus(speakerPose);
                Rotation2d targetDirection = new Rotation2d(targetTransform.getX(), targetTransform.getY());
                // Rotation2d deltaDirection = drive.getRotation().minus(targetDirection);
                
                double omega = angleController.calculate(drive.getRotation().getRadians(), targetDirection.getRadians());

                // Square values
                linearMagnitude = linearMagnitude * linearMagnitude;
                omega = Math.copySign(omega * omega, omega);

                // Calcaulate new linear velocity
                Translation2d linearVelocity =
                    new Pose2d(new Translation2d(), linearDirection)
                        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                        .getTranslation();

                // Convert to robot relative speeds & send command
                drive.runVelocity(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec(),
                        drive.getRotation()));
            },
        drive
        ) */

    /**
     * Drive robot while pointing at a specific point on the field.
     */
    public static Command joystickSpeakerPoint(
        SwerveDrive drive,
        CommandXboxController driverController) {
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        return new RunCommand(
            () -> {
                Pose2d speakerPose = new Pose2d(-0.2, (5 + 6.12)/2, new Rotation2d(0)); // make this a constant that flips for each side
                
                Transform2d targetTransform = drive.getPose().minus(speakerPose);
                Rotation2d targetDirection = new Rotation2d(targetTransform.getX(), targetTransform.getY());
                // Rotation2d deltaDirection = drive.getRotation().minus(targetDirection);
                
                double omega = angleController.calculate(drive.getRotation().getRadians(), targetDirection.getRadians());

                // Square values
                omega = Math.copySign(omega * omega, omega);
                        
                drive.drive(
                    -MathUtil.applyDeadband((1 - 0.75 * driverController.getRightTriggerAxis()) * driverController.getLeftY(), IOConstants.kDriveDeadband),
                    -MathUtil.applyDeadband((1 - 0.75 * driverController.getRightTriggerAxis()) * driverController.getLeftX(), IOConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(omega, IOConstants.kDriveDeadband),
                    true,
                    true 
            );}, drive
        );
    }
}
