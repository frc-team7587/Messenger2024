package org.metuchenmomentum.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSparkMax implements ShooterIO {
    private final CANSparkMax pivot;
    private final CANSparkMax shooter;
    private final CANSparkMax indexer;

    private final RelativeEncoder pivotEncoder;
    private final SparkPIDController pivotController;

    public ShooterSparkMax() {
        pivot = new CANSparkMax(0, MotorType.kBrushless);
        shooter = new CANSparkMax(0, MotorType.kBrushless);
        indexer = new CANSparkMax(0, MotorType.kBrushless);

        pivotEncoder = pivot.getEncoder();
        pivotController = pivot.getPIDController();
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
}
