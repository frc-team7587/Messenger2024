package org.metuchenmomentum.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.metuchenmomentum.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;


public class IntakeSparkMax implements IntakeIO {
    private final CANSparkMax pivotMotor;
    private final CANSparkMax intakeMotor;

    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotController;

    public IntakeSparkMax() {
        pivotMotor = new CANSparkMax(IntakeConstants.kIntakePivotMotorID, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);

        pivotMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();

        intakeMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder = pivotMotor.getEncoder();
        pivotController = pivotMotor.getPIDController();

        pivotController.setP(0.5);
        pivotController.setI(0);
        pivotController.setD(0);
        pivotController.setFF(0);
        pivotController.setOutputRange(-1, 1);

        pivotMotor.burnFlash();
        intakeMotor.burnFlash();
    }

    @Override
    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void setPivotSpeed(double speed) {
        pivotMotor.set(speed);
    }

    @Override
    public void setPivotPosition(double setpoint) {
        pivotController.setReference(setpoint, ControlType.kPosition);
    }

    @Override
    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    @Override
    public void turnIntake(double speed) {
        pivotMotor.set(speed);
    }
}
