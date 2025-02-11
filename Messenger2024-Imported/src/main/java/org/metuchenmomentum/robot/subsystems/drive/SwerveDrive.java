package org.metuchenmomentum.robot.subsystems.drive;

import org.metuchenmomentum.robot.Constants.DriveConstants;
import org.metuchenmomentum.robot.Constants.ModuleConstants;
import org.metuchenmomentum.robot.utils.SwerveUtils;
import org.metuchenmomentum.robot.Configs;

import com.studica.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.*;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The swerve drivetrain consisting of four independently-controlled swerve modules of turning and driving motors. */
public class SwerveDrive extends SubsystemBase {

 private RobotConfig config;


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

    private final AHRS gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);
    

    // Rate-limiting variables
    private double currentRotation = 0.0;
    private double currentTranslationDirection = 0.0;
    private double currentTranslationMagnitude = 0.0;

    private SlewRateLimiter magnitudeLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);

    private double previousTime = WPIUtilJNI.now() * 1e-6;

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(
       // DriveConstants.kDriveKinematics, getRotation(), getModulePositions()
       DriveConstants.kDriveKinematics,
       getRotation(),
       getModulePositions())
    ;
    

    public SwerveDrive() {

        
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

        

    /*/ Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        () ->
            DriveConstants.kDriveKinematics.toChassisSpeeds(
                getModuleStates()), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) ->
            driveRobotRelative(
                speeds), // Method that will drive the robot give-n ROBOT RELATIVE ChassisSpeeds.
        // Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following
            // controller for holonomic drive trains
            translationPID, // Translation PID constants
            rotationPID // Rotation PID constants
            ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
        );
        */
    }
        

    /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
    double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
  var swerveModuleStates =
      DriveConstants.kDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.discretize(
              fieldRelative
                  ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed, ySpeed, rot, gyro.getRotation2d())
                  : new ChassisSpeeds(xSpeed, ySpeed, rot),
              periodSeconds));
  SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeed);
  frontLeftModule.setDesiredState(swerveModuleStates[0]);
  frontRightModule.setDesiredState(swerveModuleStates[1]);
  rearLeftModule.setDesiredState(swerveModuleStates[2]);
  rearRightModule.setDesiredState(swerveModuleStates[3]);
}

    /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        getRotation(),
        getModulePositions());
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

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            rearLeftModule.getState(),
            rearRightModule.getState()
        };
    }

    public ChassisSpeeds getModuleSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
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
