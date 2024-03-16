// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.metuchenmomentum.robot;

import org.metuchenmomentum.robot.Constants.IOConstants;
import org.metuchenmomentum.robot.subsystems.climber.Climber;
import org.metuchenmomentum.robot.subsystems.climber.ClimberSparkMax;
import org.metuchenmomentum.robot.subsystems.drive.SwerveDrive;
import org.metuchenmomentum.robot.subsystems.intake.Intake;
import org.metuchenmomentum.robot.subsystems.intake.IntakeSparkMax;
import org.metuchenmomentum.robot.subsystems.shooter.Shooter;
import org.metuchenmomentum.robot.subsystems.shooter.ShooterSparkMax;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
    private final SwerveDrive drivetrain = new SwerveDrive();
    private final Intake intake = new Intake(new IntakeSparkMax());
    private final Shooter shooter = new Shooter(new ShooterSparkMax());
    private final Climber climber = new Climber(new ClimberSparkMax());

    CommandXboxController driverController = new CommandXboxController(IOConstants.kDriverControllerPort);
    CommandXboxController operatorController = new CommandXboxController(IOConstants.kOperatorControllerPort);

    UsbCamera camera = CameraServer.startAutomaticCapture();

    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("Intake Note", autonomousIntakeNote());
        NamedCommands.registerCommand("Handoff Note", autonomousHandoffNote());
        NamedCommands.registerCommand("Shoot Note", autonomousShootNote2());
    }

    private void configureBindings() {
        // Drive Command
        // The right trigger acts a brake so the max speed is inversely proportional to how much
        // the right trigger is held down. When it's held down completely, the maximum speed is 0.25
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
        
        /** TELEOPERATED TRIGGERS */
        operatorController.start().negate().and(operatorController.b()).toggleOnTrue(
            new SequentialCommandGroup(
               // shooter.resetPosition().withTimeout(0),
                climber.raiseLeftHook().withTimeout(0),
                climber.raiseRightHook()));
        operatorController.start().negate().and(operatorController.b()).toggleOnFalse(
            new SequentialCommandGroup(
                climber.stopLeftHook().withTimeout(0),
                climber.stopRightHook()));
           operatorController.start().negate().and(operatorController.x()).toggleOnTrue(
            new SequentialCommandGroup(
               // shooter.resetPosition().withTimeout(0),
                climber.lowerLeftHook().withTimeout(0),
                climber.lowerRightHook()));
        operatorController.start().negate().and(operatorController.x()).toggleOnFalse(
            new SequentialCommandGroup(
                climber.stopLeftHook().withTimeout(0),
                climber.stopRightHook()));
        // A: turns the intake to the ground and runs the rollers to intake the note, clicking again stops the intake
        operatorController.start().negate().and(operatorController.a()).toggleOnTrue(intake.intakeNote());
        operatorController.start().negate().and(operatorController.a()).toggleOnFalse(intake.stopIntake());

        // Y: Sets the intake the shooter to the handoff position
        operatorController.start().negate().and(operatorController.y()).toggleOnTrue(
            shooter.turnToHandoff()
                .withTimeout(0)
                .andThen(intake.turnToShooter())
        );

        // Right Trigger: starts the speaker scoring sequence
        operatorController.start().negate().and(operatorController.rightTrigger()).toggleOnTrue(
            shooter.prepareSpeakerPosition().withTimeout(.1)
                .andThen(shooter.prepareSpeaker().withTimeout(.5))
                .andThen(shooter.launchNote()
                .alongWith(intake.intakeOut()).withTimeout(1)
                .andThen(shooter.stopShooter().withTimeout(.1)))
                .andThen(intake.stopIntake().withTimeout(0).alongWith(shooter.stopIndexer()))
        );

        // POV-Up and POV-Down buttons turn the shooter to the handoff and amp positions rsespectively
        operatorController.start().negate().and(operatorController.povUp()).whileTrue(shooter.turnToHandoff());
        operatorController.start().negate().and(operatorController.povDown()).whileTrue(shooter.turnToAmp());

        // X: Resets the position
        operatorController.start().negate().and(operatorController.x()).whileTrue(
            new SequentialCommandGroup(
                shooter.resetPosition().withTimeout(0),
                intake.turnToShooter()
            )
        );

        // Left Bumper: button loads the note in the shooter so the notes doesn't touch the shooter wheels
        operatorController.start().negate().and(operatorController.leftBumper()).toggleOnTrue(
            new SequentialCommandGroup(      
                intake.releaseNoteManual().withTimeout(.5)
                .alongWith(shooter.loadNote()).withTimeout(.5),
                intake.turnToNeutral().withTimeout(.1),
                shooter.takeBackALittleBitShooter().withTimeout(.2).andThen(shooter.stopShooter()),
                shooter.stopIndexer(),
                intake.stopIntake(),
                shooter.stopShooter()
            )
        );

        // Right Bumper: button shoots the note at the current position
        operatorController.start().negate().and(operatorController.rightBumper()).toggleOnTrue(
            shooter.manualShoot().withTimeout(2).andThen(shooter.stopShooter().withTimeout(0)).andThen(shooter.stopIndexer())
        );

        // Left Trigger: starts the amp scoring sequence
        operatorController.start().negate().and(operatorController.leftTrigger()).toggleOnTrue(shooter.amplify());
    
        /** Full-Manual Mode enabled by holding the start button, commands are self-explanatory */
        operatorController.start().and(operatorController.b())
            .whileTrue(shooter.pivotUp());

        operatorController.start().and(operatorController.x())
            .whileTrue(shooter.pivotDown());

        operatorController.start().and(operatorController.y())
            .whileTrue(intake.turnToGroundManual());

        operatorController.start().and(operatorController.a())
            .whileTrue(intake.turnToShooterManual());

        operatorController.start().and(operatorController.leftTrigger())
            .toggleOnTrue(intake.intakeIn());

        operatorController.start().and(operatorController.leftTrigger())
            .toggleOnFalse(intake.stopIntake());

        operatorController.start().and(operatorController.rightTrigger())
            .toggleOnTrue(intake.intakeOut());

        operatorController.start().and(operatorController.rightTrigger())
            .toggleOnFalse(intake.stopIntake());

        operatorController.start().and(operatorController.leftBumper())
            .toggleOnTrue(shooter.prepareSpeaker());

        operatorController.start().and(operatorController.leftBumper())
            .toggleOnFalse(shooter.stopShooter());

        operatorController.start().and(operatorController.rightBumper())
            .toggleOnTrue(shooter.takeBackALittleBitShooter());

        operatorController.start().and(operatorController.rightBumper())
            .toggleOnFalse(shooter.stopShooter());

        operatorController.start().and(operatorController.povUp())
            .toggleOnTrue(shooter.loadNote());

        operatorController.start().and(operatorController.povUp())
            .toggleOnFalse(shooter.stopIndexer());

        operatorController.start().and(operatorController.povDown())
            .toggleOnTrue(shooter.takeBackALittleBitIndexer());

        operatorController.start().and(operatorController.povDown())
            .toggleOnFalse(shooter.stopIndexer());
    
    }   
    
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("3-Note Auto");
    }

    public Command autonomousIntakeNote() {
        return intake.intakeNote().withTimeout(2)
            .andThen(intake.turnToShooter().withTimeout(0.5))
            .andThen(intake.stopIntake().withTimeout(0));
    }

    public Command autonomousHandoffNote() {
        return new SequentialCommandGroup(      
            shooter.turnToHandoff().withTimeout(0.3),
            intake.releaseNoteManual().withTimeout(0.5).alongWith(shooter.loadNote()).withTimeout(0.5),
            intake.turnToNeutral().withTimeout(0.1),
            shooter.takeBackALittleBitIndexer().withTimeout(0.2).andThen(shooter.stopShooter().withTimeout(0)),
            shooter.stopIndexer().withTimeout(0),
            intake.stopIntake().withTimeout(0),
            shooter.stopShooter().withTimeout(0)
        );
    }

 /*   public Command autonomousShootNote() {
        return shooter.prepareSpeakerPosition().withTimeout(0.1)
            .andThen(shooter.prepareSpeaker().withTimeout(0.5))
            .andThen(shooter.launchNote().alongWith(intake.intakeOut()).withTimeout(1)
            .andThen(shooter.stopShooter().withTimeout(0.1)))
            .andThen(intake.stopIntake().withTimeout(0).alongWith(shooter.stopIndexer())).withTimeout(0);
    }
*/
    public Command autonomousShootNote2() {
        return new SequentialCommandGroup(      
            shooter.prepareSpeakerPosition().withTimeout(0.3),
            shooter.prepareSpeaker().withTimeout(0.5),
            shooter.launchNote().alongWith(intake.intakeOut()).withTimeout(1),
            shooter.stopShooter().withTimeout(0.1),
            shooter.stopIndexer().withTimeout(0.1),
            intake.stopIntake().withTimeout(0.1)
        );
       /* return new SequentialCommandGroup(
            // shooter.prepareSpeakerPosition(),
            shooter.prepareSpeaker().withTimeout(0.5),
            shooter.launchNote().alongWith(intake.intakeOut()).withTimeout(1),
            shooter.stopShooter().withTimeout(0.1),
            intake.stopIntake().withTimeout(0).alongWith(shooter.stopIndexer())).withTimeout(0); */
    }
}
