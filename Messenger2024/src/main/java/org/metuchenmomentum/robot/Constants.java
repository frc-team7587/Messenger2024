package org.metuchenmomentum.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class DriveConstants {
        // Allowed maximum speeds of the robot
        public static final double kMaxSpeed = 4.8; // m/s
        public static final double kMaxAngularSpeed = 2 * Math.PI; // rad/s

        public static final double kDirectionSlewRate = 1.2; // rad/s
        public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
        public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

        // Distance between centers of right and left wheels on robot
        public static final double kTrackWidth = Units.inchesToMeters(28);
        // Distance between front and back wheels on robot
        public static final double kWheelBase = Units.inchesToMeters(28);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
        );

        // Angular offsets of the modules relative to the chassis in radians
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingMotorID = 2;
        public static final int kRearLeftDrivingMotorID = 4;
        public static final int kFrontRightDrivingMotorID = 1;
        public static final int kRearRightDrivingMotorID = 3;

        public static final int kFrontLeftTurningMotorID = 6;
        public static final int kRearLeftTurningMotorID = 8;
        public static final int kFrontRightTurningMotorID = 5;
        public static final int kRearRightTurningMotorID = 7;

        public static final boolean kGyroReversed = false;
    }

    public static final class ModuleConstants {
        // The module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth
        // will result in a robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of the steering motor in the module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeed = MotorConstants.kFreeSpeed / 60;
        public static final double kWheelDiameter = 0.0762;
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheel = (kDrivingMotorFreeSpeed * kWheelCircumference) / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameter * Math.PI) / kDrivingMotorReduction; // m
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameter * Math.PI) / kDrivingMotorReduction) / 60.0; // m/s

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // rad
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // rad/s

        public static final double kTurningEncoderPositionPIDMinInput = 0; // rad
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // rad

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheel;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class IOConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kDriveDeadband = 0.05;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeed = 3; // m/s
        public static final double kMaxAcceleration = 3; // m/s/s
        public static final double kMaxAngularSpeed = Math.PI; // rad/s
        public static final double kMaxAngularAcceleration = Math.PI; // rad/s/s

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeed, kMaxAngularAcceleration);
    }

    public static final class MotorConstants {
        public static final double kFreeSpeed = 5676;
    }
}