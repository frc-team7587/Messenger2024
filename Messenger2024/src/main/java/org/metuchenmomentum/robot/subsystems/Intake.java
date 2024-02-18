package org.metuchenmomentum.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final CANSparkMax pivotMotor;
    private final CANSparkMax intakeMotor;

    public Intake() {
        pivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    }
}
