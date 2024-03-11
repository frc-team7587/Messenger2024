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
        // Drive Command
        // The right trigger acts a brake so the max speed is inversely proportional to how much
        // the right trigger is held down. When it's held down completely, 
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> drivetrain.drive(
                    -MathUtil.applyDeadband((1 - 0.75 * driverController.getRightTriggerAxis()) * driverController.getLeftY(), IOConstants.kDriveDeadband),
                    -MathUtil.applyDeadband((1 - 0.75 * driverController.getRightTriggerAxis()) * driverController.getLeftX(), IOConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(0.5 * driverController.getRightX(), IOConstants.kDriveDeadband),
                    true,
                    true 
                ), drivetrain
            )
        );  
        
            
        //main set of commands, get disabled if d-pad left
        operatorController.start().negate().and(operatorController.a()).toggleOnTrue(shooter.stopIndexer().alongWith(intake.intakeNote()));
        operatorController.start().negate().and(operatorController.a()).toggleOnFalse(intake.stopIntake());

        // Sets handoff position
        operatorController.start().negate().and(operatorController.y()).toggleOnTrue(
           shooter.turnToHandoff()
           .withTimeout(0)
           .andThen(intake.turnToShooter())
        );

        
        // Shooting from intake
        operatorController.start().negate().and(operatorController.rightTrigger()).toggleOnTrue(
            //direct shoot
                shooter.prepareSpeakerPosition().withTimeout(.3)
                .andThen(shooter.prepareSpeaker()
                .withTimeout(1.5))
                .andThen(shooter.launchNote()
                .alongWith(intake.intakeOut()).withTimeout(1)
                .andThen(shooter.stopShooter().withTimeout(.1)))
           //literally the only way to get things to stop
                .andThen(intake.stopIntake().withTimeout(0).alongWith(shooter.stopIndexer()))
        );

       

        operatorController.start().negate().and(operatorController.povUp()).whileTrue(shooter.turnToHandoff());
        operatorController.start().negate().and(operatorController.povDown()).whileTrue(shooter.turnToAmp());

        // Reset positions
        operatorController.start().negate().and(operatorController.x()).whileTrue(
            new SequentialCommandGroup(
                shooter.resetPosition().withTimeout(0),
                intake.turnToShooter()
            )
        );

        // Handoff from intake to shooter
        operatorController.start().negate().and(operatorController.leftBumper()).toggleOnTrue(
            new SequentialCommandGroup(      
        //shooter.turnToHandoff().withTimeout(1).andThen(intake.turnToShooter().withTimeout(1)),
                intake.releaseNoteManual().withTimeout(2.2)
                .alongWith(shooter.loadNote()).withTimeout(2.2),
                intake.turnToNeutral().withTimeout(.1),
                shooter.takeBackALittleBitIndexer().withTimeout(.2),
                shooter.stopIndexer(),
                intake.stopIntake()
            )
        );

        // Manual Shoot
        operatorController.start().negate().and(operatorController.rightBumper()).toggleOnTrue(
            shooter.manualShoot().withTimeout(2).andThen(shooter.stopShooter().withTimeout(0)).andThen(shooter.stopIndexer())
           
        );


        operatorController.start().negate().and(operatorController.leftTrigger()).toggleOnTrue(shooter.amplify());
    
        //safe mode commands
        operatorController.start().and(operatorController.b()).whileTrue(shooter.pivotUp());
        operatorController.start().and(operatorController.x()).whileTrue(shooter.pivotDown());
        operatorController.start().and(operatorController.y()).whileTrue(intake.turnToGroundManual());
        operatorController.start().and(operatorController.a()).whileTrue(intake.turnToShooterManual());
        operatorController.start().and(operatorController.leftTrigger()).toggleOnTrue(intake.intakeIn());
        operatorController.start().and(operatorController.leftTrigger()).toggleOnFalse(intake.stopIntake());
        operatorController.start().and(operatorController.rightTrigger()).toggleOnTrue(intake.intakeOut());
        operatorController.start().and(operatorController.rightTrigger()).toggleOnFalse(intake.stopIntake());
        operatorController.start().and(operatorController.leftBumper()).toggleOnTrue(shooter.prepareSpeaker());
        operatorController.start().and(operatorController.leftBumper()).toggleOnFalse(shooter.stopShooter());
        operatorController.start().and(operatorController.rightBumper()).toggleOnTrue(shooter.takeBackALittleBitShooter());
        operatorController.start().and(operatorController.rightBumper()).toggleOnFalse(shooter.stopShooter());
        operatorController.start().and(operatorController.povUp()).toggleOnTrue(shooter.loadNote());
        operatorController.start().and(operatorController.povUp()).toggleOnFalse(shooter.stopIndexer());
        operatorController.start().and(operatorController.povDown()).toggleOnTrue(shooter.takeBackALittleBitIndexer());
        operatorController.start().and(operatorController.povDown()).toggleOnFalse(shooter.stopIndexer());
    
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
