package org.metuchenmomentum.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO intake;

    public Intake(IntakeIO intake) {
        this.intake = intake;
    }

    public Command intakeNote() {
        return run(() -> intake.setIntakeSpeed(1));
    }

    public Command releaseNote() {
        return run(() -> intake.setIntakeSpeed(-1));
    }
}
