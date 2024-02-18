package org.metuchenmomentum.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final CANSparkMax leftClimber;
    private final CANSparkMax rightClimber;

    public Climber() {
        leftClimber = new CANSparkMax(0, MotorType.kBrushless);
        rightClimber = new CANSparkMax(0, MotorType.kBrushless);
    }
}
