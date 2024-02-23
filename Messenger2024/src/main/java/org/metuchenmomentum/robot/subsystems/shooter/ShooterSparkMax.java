package org.metuchenmomentum.robot.subsystems.shooter;

import org.metuchenmomentum.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSparkMax implements ShooterIO {
    private final CANSparkMax pivotMotor;
    private final CANSparkMax shootingMotor;
    private final CANSparkMax indexingMotor;

    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotController;

    public ShooterSparkMax() {
        pivotMotor = new CANSparkMax(ShooterConstants.kShooterPivotMotorID, MotorType.kBrushless);
        shootingMotor = new CANSparkMax(ShooterConstants.kShootingMotorID, MotorType.kBrushless);
        indexingMotor = new CANSparkMax(ShooterConstants.kIndexingMotorID, MotorType.kBrushless);

        pivotMotor.restoreFactoryDefaults();
        shootingMotor.restoreFactoryDefaults();
        indexingMotor.restoreFactoryDefaults();

        pivotMotor.setIdleMode(IdleMode.kBrake);
        shootingMotor.setIdleMode(IdleMode.kBrake);
        indexingMotor.setIdleMode(IdleMode.kBrake);

        pivotMotor.setSmartCurrentLimit(50);
        shootingMotor.setSmartCurrentLimit(80);
        indexingMotor.setSmartCurrentLimit(20);

        pivotEncoder = pivotMotor.getEncoder();
        pivotController = pivotMotor.getPIDController();

        pivotController.setP(0.5);
        pivotController.setI(0.0);
        pivotController.setD(0.1);
        pivotController.setFF(0.0);
        pivotController.setOutputRange(-1, 1);

        pivotMotor.burnFlash();
        shootingMotor.burnFlash();
        indexingMotor.burnFlash();
    }

    @Override
    public void setIndexerSpeed(double speed) {
        indexingMotor.set(speed);
    }

    @Override
    public void setShooterSpeed(double speed) {
        shootingMotor.set(speed);
    }

    @Override
    public void setShooterPosition(double target) {
        pivotController.setReference(target, ControlType.kPosition);
    }

    @Override
    public double getShooterPosition() {
        return pivotEncoder.getPosition();
    }

    @Override
    public void turnShooter(double speed) {
        pivotMotor.set(speed);
    }
}
