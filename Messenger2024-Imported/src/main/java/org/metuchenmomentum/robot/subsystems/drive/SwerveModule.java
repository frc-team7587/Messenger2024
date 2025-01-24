package org.metuchenmomentum.robot.subsystems.drive;

import org.metuchenmomentum.robot.Constants.ModuleConstants;
import org.metuchenmomentum.robot.Configs.MAXSwerveModule;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


/** A swerve module consisting of a driving and turning motor. */
public class SwerveModule implements SwerveModuleIO {
    private final SparkMax drivingMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder drivingEncoder;
    private final AbsoluteEncoder turningEncoder;

    private final SparkClosedLoopController drivingClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    private double chassisAngularOffset = 0;
    private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(int drivingMotorID, int turningMotorID, double chassisAngularOffset) {
        drivingMotor = new SparkMax(drivingMotorID, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder();

        drivingClosedLoopController = drivingMotor.getClosedLoopController();
        turningClosedLoopController = turningMotor.getClosedLoopController();

        drivingMotor.configure(MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        /*
        drivingEncoder = drivingMotor.getEncoder();
        turningEncoder = turningMotor.getAbsoluteEncoder(Type.kDutyCycle);

        drivingPIDController = drivingMotor.getPIDController();
        turningPIDController = turningMotor.getPIDController();

        drivingPIDController.setFeedbackDevice(drivingEncoder);
        turningPIDController.setFeedbackDevice(turningEncoder);

        drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
        drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

        turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        turningPIDController.setPositionPIDWrappingEnabled(true);
        turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
        turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

        drivingPIDController.setP(ModuleConstants.kDrivingP);
        drivingPIDController.setI(ModuleConstants.kDrivingI);
        drivingPIDController.setD(ModuleConstants.kDrivingD);
        drivingPIDController.setFF(ModuleConstants.kDrivingFF);
        drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);

        turningPIDController.setP(ModuleConstants.kTurningP);
        turningPIDController.setI(ModuleConstants.kTurningI);
        turningPIDController.setD(ModuleConstants.kTurningD);
        turningPIDController.setFF(ModuleConstants.kTurningFF);
        turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput);

        drivingMotor.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
        turningMotor.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
        
        drivingMotor.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
        turningMotor.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

        drivingMotor.burnFlash();
        turningMotor.burnFlash();
        */

        this.chassisAngularOffset = chassisAngularOffset;
        desiredState.angle = new Rotation2d(turningEncoder.getPosition());
        drivingEncoder.setPosition(0);
    }

    @Override
    public double getDrivingPosition() {
        return drivingEncoder.getPosition();
    }

    @Override
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    @Override
    public double getDrivingVelocity() {
        return drivingEncoder.getVelocity();
    }

    @Override
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDrivingVelocity(), new Rotation2d(getTurningPosition() - chassisAngularOffset));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivingPosition(), new Rotation2d(getTurningPosition() - chassisAngularOffset));
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, new Rotation2d(getTurningPosition()));
        
        drivingClosedLoopController.setReference(optimizedDesiredState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        turningClosedLoopController.setReference(optimizedDesiredState.angle.getRadians(), SparkMax.ControlType.kPosition);

        this.desiredState = desiredState;
    }

    @Override
    public void resetEncoder() {
        drivingEncoder.setPosition(0);
    }
}
