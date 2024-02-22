package org.metuchenmomentum.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final ClimberIO climber;

    public Climber(ClimberIO climber) {
        this.climber = climber;
    }

    public Command raiseLeftHook() {
        return run(() -> climber.setLeftHookSpeed(1));
    }

    public Command raiseRightHook() {
        return run(() -> climber.setRightHookSpeed(1));
    }

    public Command lowerLeftHook() {
        return run(() -> climber.setLeftHookSpeed(-1));
    }

    public Command lowerRightHook() {
        return run(() -> climber.setRightHookSpeed(-1));
    }
}
