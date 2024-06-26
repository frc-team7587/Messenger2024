package org.metuchenmomentum.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO intake;

    public Intake(IntakeIO intake) {
        this.intake = intake;
    }

    public Command intakeNote() {
        return new SequentialCommandGroup(
            turnToGround().withTimeout(0),
            intakeIn()
        );

    }

    // runs the intake
    public Command intakeIn() {
        return run(
            () -> intake.setIntakeSpeed(IntakeConstants.kIntakeInSpeed));
    }

    // runs intake in opposite direction
    public Command intakeOut() {
        return run(
            () -> intake.setIntakeSpeed(-IntakeConstants.kIntakeInSpeed));
    }

    // runs the intake, stops when command is no longer called
    public Command intakeNoteManual() {
        return startEnd(
            () -> intake.setIntakeSpeed(IntakeConstants.kIntakeInSpeed),
            () -> intake.setIntakeSpeed(0));
    }

    // runs the intake in opposite direction, stops when command is no longer called
    public Command releaseNoteManual() {
        return startEnd(
            () -> intake.setIntakeSpeed(IntakeConstants.kIntakeOutSpeed),
            () -> intake.setIntakeSpeed(0));
    }

    // turns the intake towards the ground while command is called
    public Command turnToGroundManual() {
        return new Command() {
            @Override
            public void execute() {
                intake.setPivotSpeed(IntakeConstants.kPivotUpSpeed);
            }

            @Override
            public void end(boolean interrupted) {
                intake.setPivotSpeed(0);
            }
        };
    }

    // turns the intake towards the shooter while command is called
    public Command turnToShooterManual() {
        return new Command() {
            @Override
            public void execute() {
                intake.setPivotSpeed(IntakeConstants.kPivotDownSpeed);
            }

            @Override
            public void end(boolean interrupted) {
                intake.setPivotSpeed(0);
            }
        };
    }
    
    // turns the intake towards the shooter angle
    public Command turnToShooter() {
        return run(() -> intake.setPivotPosition(IntakeConstants.kIntakeShooterPosition));
    }
    
    // turns the intake towards the ground angle
    public Command turnToGround() {
        return run(() -> intake.setPivotPosition(IntakeConstants.kIntakeGroundPosition));
    }

    // turns the intake towards the neutral angle to allow handoff
    public Command turnToNeutral() {
        return run(() -> intake.setPivotPosition(IntakeConstants.kIntakeNeutralPosition));
    }

    public void resetEncoder() {
        intake.reset();
    }

    // stops the intake wheels
    public Command stopIntake(){
        return run(() -> intake.setIntakeSpeed(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", Math.round(intake.getPivotPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }
}