package org.metuchenmomentum.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final CANSparkMax pivotMotor;
    private final CANSparkMax shootingMotor;
    private final CANSparkMax indexingMotor;

    public Shooter() {
        pivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        shootingMotor = new CANSparkMax(0, MotorType.kBrushless);
        indexingMotor = new CANSparkMax(0, MotorType.kBrushless);
    }
}
