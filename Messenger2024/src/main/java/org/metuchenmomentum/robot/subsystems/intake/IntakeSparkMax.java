package org.metuchenmomentum.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSparkMax extends SubsystemBase {
    private final CANSparkMax pivotMotor;
    private final CANSparkMax intakeMotor;

    public IntakeSparkMax() {
        pivotMotor = new CANSparkMax(0, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(0, MotorType.kBrushless);
    }
}
