package org.metuchenmomentum.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    }

    @Override
    public void raiseLeftHook() {
        leftHook.set(1);
    }

    @Override
    public void raiseRightHook() {
        rightHook.set(1);
    }

    @Override
    public void lowerLeftHook() {
        leftHook.set(-1);
    }

    @Override
    public void lowerRightHook() {
        rightHook.set(-1);
    }

    @Override
    public void raiseHooks() {
        raiseLeftHook();
        raiseRightHook();
    }

    @Override
    public void lowerHooks() {
        lowerLeftHook();
        lowerRightHook();
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
