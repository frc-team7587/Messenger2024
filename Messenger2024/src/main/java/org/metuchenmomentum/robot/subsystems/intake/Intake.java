package org.metuchenmomentum.robot.subsystems.intake;

import org.metuchenmomentum.robot.Constants.IntakeConstants;

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
}
