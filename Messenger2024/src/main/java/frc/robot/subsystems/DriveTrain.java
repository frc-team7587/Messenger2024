package frc.robot.subsystems;

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

import frc.robot.Constants.DriveConstants;
import frc.robot.utils.SwerveUtils;

import com.kauailabs.navx.frc.AHRS;

public class DriveTrain extends SubsystemBase {
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

    // Slew rate filter variables for controlling lateral acceleration
    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double previousTime = WPIUtilJNI.now() * 1e-6;

    // Odometry class for tracking robot pose
    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            rearLeftModule.getPosition(),
            rearRightModule.getPosition()
        }
    );

    /** Creates a new DriveSubsystem. */
    public DriveTrain() {
        resetHeading();
    }

    /**
     * Drives the robot.
     * @param xSpeed                Speed of the robot in the x direction (forward).
     * @param ySpeed                Speed of the robot in the y direction (sideways).
     * @param rotation              Angular rate of the robot.
     * @param isFieldRelative       Whether the provided x and y speeds are relative to the field.
     * @param isRateLimitEnabled    Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rotation, boolean isFieldRelative, boolean isRateLimitEnabled) {
        double xSpeedCommand;
        double ySpeedCommand;

        if (isRateLimitEnabled) {
            // Convert XY to polar for rate limiting
            double inputTranslationDirection = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMagnitude = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral acceleration
            double directionSlewRate;
            if (currentTranslationMagnitude != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMagnitude);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
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
                if (currentTranslationMagnitude > 1e-4) { // some small number to avoid floating-point errors with equality checking
                    // currentTranslationDirection is unchanged
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

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommand * DriveConstants.kMaxSpeed;
        double ySpeedDelivered = ySpeedCommand * DriveConstants.kMaxSpeed;
        double rotationDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            isFieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered));
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
        frontLeftModule.setDesiredState(swerveModuleStates[0]);
        frontRightModule.setDesiredState(swerveModuleStates[1]);
        rearLeftModule.setDesiredState(swerveModuleStates[2]);
        rearRightModule.setDesiredState(swerveModuleStates[3]);
    }

    @Override
    public void periodic() {
        odometry.update(
            Rotation2d.fromDegrees(getAngle()),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                rearLeftModule.getPosition(),
                rearRightModule.getPosition()
            }
        );
    }

    /**
     * 
     * @return The angle of the robot.
     */
    public double getAngle() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            Rotation2d.fromDegrees(getAngle()),
            new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                rearLeftModule.getPosition(),
                rearRightModule.getPosition()
            },
            pose
        );
    }

    /** Sets the wheels into an X formation to prevent movement. */
    public void setX() {
        frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        rearRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeed);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        rearLeftModule.setDesiredState(desiredStates[2]);
        rearRightModule.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        frontLeftModule.resetEncoder();
        rearLeftModule.resetEncoder();
        frontRightModule.resetEncoder();
        rearRightModule.resetEncoder();
    }

    /** Resets the heading of the robot. */
    public void resetHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Rotation2d.fromDegrees(getAngle()).getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }
}
