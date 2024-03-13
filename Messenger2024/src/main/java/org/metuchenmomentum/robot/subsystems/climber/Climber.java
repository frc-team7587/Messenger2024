package org.metuchenmomentum.robot.subsystems.climber;

import org.metuchenmomentum.robot.Constants.ClimberConstants;
import org.metuchenmomentum.robot.subsystems.intake.IntakeConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final ClimberIO climber;

    public Climber(ClimberIO climber) {
        this.climber = climber;
    }

    public Command raiseLeftHook() {
        return run(
            () -> climber.setLeftHookSpeed(ClimberConstants.kLeftHookRaiseSpeed));
    }
    

    public Command raiseRightHook() {
        return run(
            () -> climber.setRightHookSpeed(-ClimberConstants.kLeftHookRaiseSpeed)
          
        );
    }

    public Command lowerLeftHook() {
        return run(
            () -> climber.setLeftHookSpeed(ClimberConstants.kLeftHookLowerSpeed)
          
        );
    }

    public Command lowerRightHook() {
        return run(
            () -> climber.setRightHookSpeed(ClimberConstants.kRightHookLowerSpeed)
            
        );
    }
    public Command stopLeftHook() {
        return run(
            () -> climber.setLeftHookSpeed(0)
        );
    }
    public Command stopRightHook() {
        return run(
            () -> climber.setRightHookSpeed(0)
        );
    }
}
