package org.metuchenmomentum.robot.subsystems.shooter;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

public class ShooterSparkMax implements ShooterIO {
    private final SparkMax pivot;
    private final SparkMax shooter;
    private final SparkMax indexer;

    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotController;

    public ShooterSparkMax() {
        pivot = new SparkMax(ShooterConstants.kShooterPivotMotorID, MotorType.kBrushless);
        shooter = new SparkMax(ShooterConstants.kShootingMotorID, MotorType.kBrushless);
        indexer = new SparkMax(ShooterConstants.kIndexingMotorID, MotorType.kBrushless);

    

        pivotEncoder = pivot.getEncoder();
        pivotController = pivot.getClosedLoopController();

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
/* 
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
*/
    @Override
    public void reset() {
        pivotEncoder.setPosition(0);
    }
}
