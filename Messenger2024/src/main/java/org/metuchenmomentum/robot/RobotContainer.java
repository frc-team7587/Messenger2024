// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.metuchenmomentum.robot;

import java.util.List;

import org.metuchenmomentum.robot.Constants.AutoConstants;
import org.metuchenmomentum.robot.Constants.DriveConstants;
import org.metuchenmomentum.robot.Constants.IOConstants;

import org.metuchenmomentum.robot.subsystems.drive.SwerveDrive;
import org.metuchenmomentum.robot.subsystems.intake.Intake;
import org.metuchenmomentum.robot.subsystems.intake.IntakeSparkMax;
import org.metuchenmomentum.robot.subsystems.shooter.Shooter;
import org.metuchenmomentum.robot.subsystems.shooter.ShooterSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private final SwerveDrive drivetrain = new SwerveDrive();
    private final Intake intake = new Intake(new IntakeSparkMax());
    private final Shooter shooter = new Shooter(new ShooterSparkMax());

    CommandXboxController driverController = new CommandXboxController(IOConstants.kDriverControllerPort);
    CommandXboxController operatorController = new CommandXboxController(IOConstants.kOperatorControllerPort);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.drive(
                    -MathUtil.applyDeadband(0.5 * driverController.getLeftY(), IOConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(0.5 * driverController.getLeftX(), IOConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(0.5 * driverController.getRightX(), IOConstants.kDriveDeadband),
                    true,
                    true 
                ), drivetrain
            )
        );
        
        driverController.a().toggleOnTrue(intake.intakeNote());
        driverController.a().toggleOnFalse(intake.stopIntake());
        driverController.y().toggleOnTrue(
           shooter.turnToHandoff()
           .withTimeout(0)
           .andThen(intake.turnToShooter())
        );
        //driverController.rightBumper().whileTrue(intake.releaseNoteManual());
        


        driverController.start().toggleOnTrue(
           //direct shoot
            shooter.prepareSpeaker()
            .withTimeout(1.5)
            .andThen(shooter.launchNote().alongWith(intake.intakeOut()).withTimeout(1).andThen(shooter.stopShooter().withTimeout(.1)))
            .andThen(intake.stopIntake().alongWith(shooter.stopIndexer()))
            );
      //  driverController.start().toggleOnFalse(new SequentialCommandGroup(
        //    shooter.stopIndexer(),
          //  shooter.stopShooter(),
            //intake.stopIntake()
            //));
        driverController.b().whileTrue(shooter.pivotUp());
        driverController.x().whileTrue(shooter.pivotDown());

        driverController.povUp().whileTrue(shooter.turnToHandoff());
        driverController.povRight().whileTrue(shooter.turnToAmp());
        driverController.povDown().whileTrue(shooter.resetPosition());

        driverController.povLeft().toggleOnTrue(new SequentialCommandGroup(
        //DoHandoff       
        //shooter.turnToHandoff().withTimeout(1).andThen(intake.turnToShooter().withTimeout(1)),
                intake.releaseNoteManual().withTimeout(2).alongWith(shooter.loadNote()).withTimeout(2),
                shooter.takeBackALittleBitIndexer().withTimeout(.1),
                shooter.stopIndexer(),
                intake.stopIntake()
            )
        );

        driverController.rightTrigger().toggleOnTrue(
            shooter.manualShoot().withTimeout(2).andThen(shooter.stopShooter())
            //manualShoot
            //shooter.prepareSpeaker()
           // .withTimeout(1.5)
          //  .andThen(shooter.launchNote())
        //    .handleInterrupt(() -> shooter.stopIndexer().alongWith(shooter.stopShooter()))
        );
        driverController.rightTrigger().toggleOnFalse(new SequentialCommandGroup(
        shooter.stopIndexer(),
        shooter.stopShooter()
        ));
        
        driverController.leftTrigger().toggleOnTrue(shooter.amplify());
    }

    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeed,
            AutoConstants.kMaxAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            exampleTrajectory,
            drivetrain::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            drivetrain::setModuleStates,
            drivetrain);

        // Reset odometry to the starting pose of the trajectory.
        drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> drivetrain.drive(0, 0, 0, false, false));
    }
}
