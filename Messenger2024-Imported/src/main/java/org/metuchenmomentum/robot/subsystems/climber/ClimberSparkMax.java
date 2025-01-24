package org.metuchenmomentum.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import org.metuchenmomentum.robot.Configs;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.*;

public class ClimberSparkMax implements ClimberIO {
    private final SparkMax leftHook;
    private final SparkMax rightHook;

    private final RelativeEncoder leftHookEncoder;
    private final RelativeEncoder rightHookEncoder;

    public ClimberSparkMax() {
        leftHook = new SparkMax(14, MotorType.kBrushless);
        rightHook = new SparkMax(15, MotorType.kBrushless);

        leftHookEncoder = leftHook.getEncoder();
        rightHookEncoder = rightHook.getEncoder();


        leftHook.configure(Configs.ClimberConfigs.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightHook.configure(Configs.ClimberConfigs.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void setLeftHookSpeed(double speed) {
        leftHook.set(speed);
    }

    @Override
    public void setRightHookSpeed(double speed) {
        rightHook.set(speed);
    }

    @Override
    public void setHookSpeeds(double speed) {
        leftHook.set(speed);
        rightHook.set(speed);
    }

    @Override
    public double getLeftHookHeight() {
        return leftHookEncoder.getPosition();
    }

    @Override
    public double getRightHookHeight() {
        return rightHookEncoder.getPosition();
    }
}
