// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.metuchenmomentum.robot;

import org.metuchenmomentum.robot.Constants.DriveConstants;
import org.metuchenmomentum.robot.Constants.IOConstants;
import org.metuchenmomentum.robot.subsystems.drive.SwerveDrive;
import org.metuchenmomentum.robot.subsystems.intake.Intake;
import org.metuchenmomentum.robot.subsystems.intake.IntakeSparkMax;
import org.metuchenmomentum.robot.subsystems.shooter.Shooter;
import org.metuchenmomentum.robot.subsystems.shooter.ShooterSparkMax;
import org.metuchenmomentum.robot.subsystems.vision.LimelightHelpers;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RobotContainer {
    //subsystems
    private final SwerveDrive drivetrain = new SwerveDrive();
    private final Intake intake = new Intake(new IntakeSparkMax());
    private final Shooter shooter = new Shooter(new ShooterSparkMax());
    // private final Climber climber = new Climber(new ClimberSparkMax());

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    //controllers
    XboxController driverController = new XboxController(IOConstants.kDriverControllerPort);
  //  XboxController operatorController = new XboxController(IOConstants.kOperatorControllerPort);

    //cameras
    UsbCamera camera = CameraServer.startAutomaticCapture();

    public RobotContainer() {
        configureBindings();

       // NamedCommands.registerCommand("Intake Note", autonomousIntakeNote());
        //NamedCommands.registerCommand("Handoff Note", autonomousHandoffNote());
      //  NamedCommands.registerCommand("Shoot Note", autonomousShootNote());
        // NamedCommands.registerCommand("Lower Climbers", autonomousLowerClimber());
    }

  public void autonomousPeriodic() {
    drive(false);
    drivetrain.updateOdometry();
  }


  public void teleopPeriodic() {
    drive(true);
  }
    private void configureBindings() {
        
        // Drive Command
        // The right trigger acts a brake so the max speed is inversely proportional to how much
        /*/ the right trigger is held down. When it's held down completely, the maximum speed is 0.25
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
        );*/
        
        /** TELEOPERATED TRIGGERS */

      /*  // Climber Controls
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
                 */

        /* 
        // A: turns the intake to the ground and runs the rollers to intake the note, clicking again stops the intake
        operatorController.start().negate().and(operatorController.a()).toggleOnTrue(
            shooter.stopShooter().withTimeout(0).andThen(shooter.stopIndexer().withTimeout(0)).andThen(intake.intakeNote()));
        operatorController.start().negate().and(operatorController.a()).toggleOnFalse(intake.stopIntake());

        // Y: Sets the intake the shooter to the handoff position
        operatorController.start().negate().and(operatorController.y()).toggleOnTrue(
            shooter.turnToHandoff()
                .withTimeout(0)
                .andThen(intake.turnToShooter())
        );
        operatorController.start().negate().and(operatorController.b()).toggleOnTrue(
        shooter.prepareSpeakerPosition().withTimeout(.2)
        .andThen(shooter.takeBackALittleBitShooter())
        );
        operatorController.start().negate().and(operatorController.b()).toggleOnFalse(
shooter.stopShooter()
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
                intake.turnToShooter().withTimeout(0),
                intake.stopIntake().withTimeout(0),
                shooter.stopIndexer().withTimeout(0),
                shooter.stopShooter().withTimeout(0)
            )
        );

        // Left Bumper: button loads the note in the shooter so the notes doesn't touch the shooter wheels
        operatorController.start().negate().and(operatorController.leftBumper()).toggleOnTrue(
            new SequentialCommandGroup(
                intake.turnToShooter().alongWith(shooter.turnToHandoff()).withTimeout(.5),
                intake.intakeIn().withTimeout(.5),      
                intake.releaseNoteManual().withTimeout(.5)
                .alongWith(shooter.loadNote()).withTimeout(.75),
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
    
        /** Full-Manual Mode enabled by holding the start button, commands are self-explanatory *
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
    */
    }  
    
  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= DriveConstants.kMaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftY(), 0.02))
            * DriveConstants.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driverController.getLeftX(), 0.02))
            * DriveConstants.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(driverController.getRightX(), 0.02))
            * DriveConstants.kMaxAngularSpeed;

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    if(driverController.getAButton())
    {
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        //while using Limelight, turn off field-relative driving.
        fieldRelative = false;
    }

    drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative, Robot.getPeriod);
  } 
    
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Shoot Only");
    }

    // public Command autonomousLowerClimber() {
    //     return climber.lowerLeftHook().withTimeout(0).andThen(climber.lowerRightHook()).withTimeout(3);
    // }

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

    public Command autonomousShootNote() {
        return new SequentialCommandGroup(      
            intake.turnToNeutral().withTimeout(0.1),
            shooter.prepareSpeakerPosition().withTimeout(0.3),
            shooter.prepareSpeaker().withTimeout(0.5),
            shooter.launchNote().alongWith(intake.intakeOut()).withTimeout(1),
            shooter.stopShooter().withTimeout(0),
            shooter.stopIndexer().withTimeout(0),
            intake.stopIntake().withTimeout(0)
        );
    }
}
