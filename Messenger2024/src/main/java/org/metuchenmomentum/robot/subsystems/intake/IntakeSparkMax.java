package org.metuchenmomentum.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;


public class IntakeSparkMax implements IntakeIO {
    private final CANSparkMax pivot;
    private final CANSparkMax intake;

    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotController;

    public IntakeSparkMax() {
        pivot = new CANSparkMax(0, MotorType.kBrushless);
        intake = new CANSparkMax(0, MotorType.kBrushless);

        pivotEncoder = pivot.getEncoder();
        pivotController = pivot.getPIDController();

        intake.setIdleMode(IdleMode.kBrake);
        pivot.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intake.set(speed);
    }

    @Override
    public void setPivotSpeed(double speed) {
        pivot.set(speed);
    }

    @Override
    public void setPivotPosition(double target) {
        pivotController.setReference(target, ControlType.kPosition);
    }

    @Override
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }
}
