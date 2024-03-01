package org.metuchenmomentum.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO intake;

    public Intake(IntakeIO intake) {
        this.intake = intake;
    }

    public Command intakeNote() {
        return run(() -> intake.setIntakeSpeed(IntakeConstants.kIntakeInSpeed));
    }

    public Command releaseNote() {
        return run(() -> intake.setIntakeSpeed(IntakeConstants.kIntakeOutSpeed));
    }

    public Command turnToGroundManual() {
        return run(() -> intake.turnIntake(IntakeConstants.kPivotDownSpeed));
    }

    public Command turnToShooterManual() {
        return run(() -> intake.turnIntake(IntakeConstants.kPivotUpSpeed));
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