package org.metuchenmomentum.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

public class ClimberSparkMax implements ClimberIO {
    private final CANSparkMax leftHook;
    private final CANSparkMax rightHook;

    private final RelativeEncoder leftHookEncoder;
    private final RelativeEncoder rightHookEncoder;

    public ClimberSparkMax() {
        leftHook = new CANSparkMax(0, MotorType.kBrushless);
        rightHook = new CANSparkMax(0, MotorType.kBrushless);

        leftHookEncoder = leftHook.getEncoder();
        rightHookEncoder = rightHook.getEncoder();

        leftHook.setIdleMode(IdleMode.kBrake);
        rightHook.setIdleMode(IdleMode.kBrake);
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
