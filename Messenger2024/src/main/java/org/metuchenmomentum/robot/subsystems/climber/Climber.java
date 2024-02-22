package org.metuchenmomentum.robot.subsystems.climber;

import org.metuchenmomentum.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final ClimberIO climber;

    public Climber(ClimberIO climber) {
        this.climber = climber;
    }

    public Command raiseLeftHook() {
        return startEnd(
            () -> climber.setLeftHookSpeed(ClimberConstants.kLeftHookRaiseSpeed),
            () -> climber.setLeftHookSpeed(0)
        );
    }

    public Command raiseRightHook() {
        return startEnd(
            () -> climber.setRightHookSpeed(ClimberConstants.kRightHookRaiseSpeed),
            () -> climber.setRightHookSpeed(0)
        );
    }

    public Command lowerLeftHook() {
        return startEnd(
            () -> climber.setLeftHookSpeed(ClimberConstants.kLeftHookLowerSpeed),
            () -> climber.setLeftHookSpeed(0)
        );
    }

    public Command lowerRightHook() {
        return startEnd(
            () -> climber.setRightHookSpeed(ClimberConstants.kRightHookLowerSpeed),
            () -> climber.setRightHookSpeed(0)
        );
    }
}
