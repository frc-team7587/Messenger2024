package org.metuchenmomentum.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
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

        setP(IntakeConstants.kP);
        setI(IntakeConstants.kI);
        setD(IntakeConstants.kD);
        pivotController.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);

        // pivotMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
        pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) IntakeConstants.kIntakeGroundPosition);
        pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        pivotMotor.setSoftLimit(SoftLimitDirection.kForward, (float) IntakeConstants.kIntakeShooterPosition);
        pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);


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
    public void setP(double p) {
        pivotController.setP(p);
    }

    @Override
    public void setI(double i) {
        pivotController.setI(i);
    }

    @Override
    public void setD(double d) {
        pivotController.setD(d);
    }

    @Override
    public double getP() {
        return pivotController.getP();
    }

    @Override
    public double getI() {
        return pivotController.getI();
    }

    @Override
    public double getD() {
        return pivotController.getD();
    }

    @Override
    public void reset() {
        pivotEncoder.setPosition(0);
    }
}
