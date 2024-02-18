package org.metuchenmomentum.robot.subsystems.drive;

import org.metuchenmomentum.robot.Constants.DriveConstants;
import org.metuchenmomentum.robot.utils.SwerveUtils;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The swerve drivetrain consisting of four independently-controlled swerve modules of turning and driving motors. */
public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeftModule = new SwerveModule(
        DriveConstants.kFrontLeftDrivingMotorID,
        DriveConstants.kFrontLeftTurningMotorID,
        DriveConstants.kFrontLeftChassisAngularOffset);

    private final SwerveModule rearLeftModule = new SwerveModule(
        DriveConstants.kRearLeftDrivingMotorID,
        DriveConstants.kRearLeftTurningMotorID,
        DriveConstants.kBackLeftChassisAngularOffset);

    private final SwerveModule frontRightModule = new SwerveModule(
        DriveConstants.kFrontRightDrivingMotorID,
        DriveConstants.kFrontRightTurningMotorID,
        DriveConstants.kFrontRightChassisAngularOffset);

    private final SwerveModule rearRightModule = new SwerveModule(
        DriveConstants.kRearRightDrivingMotorID,
        DriveConstants.kRearRightTurningMotorID,
        DriveConstants.kBackRightChassisAngularOffset);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    // Rate-limiting variables
    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

    private double previousTime = WPIUtilJNI.now() * 1e-6;

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics, getRotation(), getModulePositions()
    );

    public SwerveDrive() {
        resetHeading();
    }


    /**
     * Drives the robot.
     * @param xSpeed                    The input speed of the robot in the x direction (forwards).
     * @param ySpeed                    The input speed of the robot in the y direction (sideways).
     * @param rotation                  The input rotational speed of the robot.
     * @param isFieldRelative           Whether the provided x and y speeds are relative to the field.
     * @param isRateLimitingEnabled     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean isFieldRelative, boolean isRateLimitingEnabled) {
        double xSpeedCommand;
        double ySpeedCommand;

        // This rate-limiting algorithm is provided by REV for the MAXSwerve modules.
        if (isRateLimitingEnabled) {
            double inputTranslationDirection = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            double directionSlewRate;
            if (currentTranslationMagnitude != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMagnitude);
            } else {
                directionSlewRate = 500.0;
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - previousTime;
            double angleDifference = SwerveUtils.angleDifference(inputTranslationDirection, currentTranslationDirection);

            if (angleDifference < 0.45 * Math.PI) {
                currentTranslationDirection = SwerveUtils.stepTowardsCircular(
                    currentTranslationDirection,
                    inputTranslationDirection,
                    directionSlewRate * elapsedTime
                );
                currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
            } else if (angleDifference > 0.85 * Math.PI) {
                if (currentTranslationMagnitude > 1e-4) {
                    currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
                } else {
                    currentTranslationDirection = SwerveUtils.wrapAngle(currentTranslationDirection + Math.PI);
                    currentTranslationMagnitude = magnitudeLimiter.calculate(inputTranslationMagnitude);
                }
            } else {
                currentTranslationDirection = SwerveUtils.stepTowardsCircular(
                    currentTranslationDirection,
                    inputTranslationDirection,
                    directionSlewRate * elapsedTime
                );
                currentTranslationMagnitude = magnitudeLimiter.calculate(0.0);
            }
            previousTime = currentTime;

            xSpeedCommand = currentTranslationMagnitude * Math.cos(currentTranslationDirection);
            ySpeedCommand = currentTranslationMagnitude * Math.sin(currentTranslationDirection);
            currentRotation = rotationLimiter.calculate(rotation);

        } else {
            xSpeedCommand = xSpeed;
            ySpeedCommand = ySpeed;
            currentRotation = rotation;
        }

        double xSpeedDelivered = xSpeedCommand * DriveConstants.kMaxSpeed;
        double ySpeedDelivered = ySpeedCommand * DriveConstants.kMaxSpeed;
        double rotationDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

        SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered, getRotation())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered));
        
        setModuleStates(desiredStates);
    }

    @Override
    public void periodic() {
        odometry.update(getRotation(), getModulePositions());
    }

    /** 
     * Returns the heading of the robot reported by the gyroscope.
     * @return The heading of the robot.
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    /**
     * Returns the rotation of the robot reported by the gyroscope.
     * @return The rotation of the robot.
     */
    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }

    /**
     * Returns the pose of the robot.
     * @return The pose of the robot.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }


    /**
     * Returns the positions of the swerve modules.
     * @return The swerve module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        };
    }

    /**
     * Sets the swerve modules to their desired states.
     * @param desiredStates The states to set the swerve modules.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeed);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        rearLeftModule.setDesiredState(desiredStates[2]);
        rearRightModule.setDesiredState(desiredStates[3]);
    }

    /**
     * Resets the odometry of the robot.
     * @param pose The pose to set the odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation(), getModulePositions(), pose);
    }

    /**
     * Resets the driving encoders of the modules.
     */
    public void resetEncoders() {
        frontLeftModule.resetEncoder();
        rearLeftModule.resetEncoder();
        frontRightModule.resetEncoder();
        rearRightModule.resetEncoder();
    }

    /**
     * Resets the gyroscope's z-axis (yaw) to a heading of zero.
     */
    public void resetHeading() {
        gyro.reset();
    }
}
