package org.metuchenmomentum.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSparkMax implements ShooterIO {
    private final CANSparkMax pivot;
    private final CANSparkMax shooter;
    private final CANSparkMax indexer;

    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotController;

    public ShooterSparkMax() {
        pivot = new CANSparkMax(ShooterConstants.kShooterPivotMotorID, MotorType.kBrushless);
        shooter = new CANSparkMax(ShooterConstants.kShootingMotorID, MotorType.kBrushless);
        indexer = new CANSparkMax(ShooterConstants.kIndexingMotorID, MotorType.kBrushless);

        pivot.restoreFactoryDefaults();
        shooter.restoreFactoryDefaults();
        indexer.restoreFactoryDefaults();

        pivot.setIdleMode(IdleMode.kBrake);
        shooter.setIdleMode(IdleMode.kBrake);
        indexer.setIdleMode(IdleMode.kBrake);

        pivot.setSmartCurrentLimit(50);
        shooter.setSmartCurrentLimit(80);
        indexer.setSmartCurrentLimit(20);

        pivotEncoder = pivot.getEncoder();
        pivotController = pivot.getPIDController();

        setP(ShooterConstants.kP);
        setI(ShooterConstants.kI);
        setD(ShooterConstants.kD);
        pivotController.setOutputRange(ShooterConstants.kMinOutput, ShooterConstants.kMaxOutput);

        pivot.burnFlash();
        shooter.burnFlash();
        indexer.burnFlash();
    }

    @Override
    public void setIndexerSpeed(double speed) {
        indexer.set(speed);
    }

    @Override
    public void setShooterSpeed(double speed) {
        shooter.set(speed);
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
        pivot.set(speed);
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
