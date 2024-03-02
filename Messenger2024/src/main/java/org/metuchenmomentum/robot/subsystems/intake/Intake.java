package org.metuchenmomentum.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO intake;

    public Intake(IntakeIO intake) {
        this.intake = intake;
    }

    public Command intakeNote() {
        return startEnd(
            () -> intake.setIntakeSpeed(IntakeConstants.kIntakeInSpeed),
            () -> intake.setIntakeSpeed(0));
    }

    public Command releaseNote() {
        return startEnd(
            () -> intake.setIntakeSpeed(IntakeConstants.kIntakeOutSpeed),
            () -> intake.setIntakeSpeed(0));
    }

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

    public Command turnToGround() {
        return run(() -> intake.setPivotPosition(180));
    }

    public Command turnToShooter() {
        return run(() -> intake.setPivotPosition(0));
    }

    public Command turnToNeutral() {
        return run(() -> intake.setPivotPosition(90));
    }
}