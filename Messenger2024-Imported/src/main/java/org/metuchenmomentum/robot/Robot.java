package org.metuchenmomentum.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.Optional;

import org.metuchenmomentum.robot.Constants.DriveConstants;
import org.metuchenmomentum.robot.subsystems.drive.SwerveDrive;
import org.metuchenmomentum.robot.subsystems.marquee.*;
import org.metuchenmomentum.robot.subsystems.vision.LimelightHelpers;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    //marquee
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private Marquee displayDriver;
    private DigitalOutput digOut0;
    //display sponsors
    private DisplaySponsors displaySponsors;
    public static double getPeriod;

    private final XboxController m_controller = new XboxController(0);
  private final SwerveDrive m_swerve = new SwerveDrive();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        //marquee
       // m_chooser.setDefaultOption("Default Auto", kDefaultAuto); 
      //  m_chooser.addOption("My Auto", kCustomAuto);
      //  SmartDashboard.putData("Auto choices", m_chooser);
        digOut0 = new DigitalOutput(0);
        digOut0.setPWMRate(1000);
        digOut0.enablePWM(0.5);
        DisplayConnection displayConnection = DisplayConnectionFactory.usbConnection();
        //displayDriver = new Marquee(DisplayConnectionFactory.usbConnection());
        //marquee
        //displayDriver.robotPeriodic();
        
        //display sponsors
        Optional<Alliance> ally = DriverStation.getAlliance();
          if (ally.get() == Alliance.Red){
            Sponsors sponsors = new Sponsors("Red");
           displaySponsors = new DisplaySponsors(sponsors.getList(), displayConnection, "Go Red!!!");
            displaySponsors.init();
          }
          if (ally.get() == Alliance.Blue) {
            Sponsors sponsors = new Sponsors("Blue");
            displaySponsors = new DisplaySponsors(sponsors.getList(), displayConnection, "Go Blue!!!");
            displaySponsors.init();
          }
        
    }

    @Override
    public void robotPeriodic() {
        
        CommandScheduler.getInstance().run();
        displaySponsors.robotPeriodic();
        getPeriod = this.getPeriod();
   
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void disabledExit() {

    }

    @Override
    public void autonomousInit() {
        /*
        autonomousCommand = robotContainer.getAutonomousCommand();
        //marquee
        m_autoSelected = m_chooser.getSelected();
        //displayDriver.autonomousInit();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
            */
    }

    @Override
    public void autonomousPeriodic() {
        drive(true);
        m_swerve.updateOdometry();
        /*/marquee PLEASE DONT TOUCH COMMENTED OUT CODE
        switch (m_autoSelected) {
            case kCustomAuto:
                displayDriver.autonomousPeriodic();
                break;
            case kDefaultAuto:
                displayDriver.autonomousPeriodic();
                break;
        }*/
    }

    @Override
    public void autonomousExit() {

    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

  @Override
  public void teleopPeriodic() {
    drive(false);
  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .015;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= DriveConstants.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = 0.5;
    double targetingForwardSpeed = LimelightHelpers.getTA("limelight") * kP;
    targetingForwardSpeed *= DriveConstants.kMaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  double limelight_distance(){
    double limelightMountAngleDegrees = 3.3;
    double limelightLensHeightInches = 9;
    double goalHeightInches = 13;
    double kP = 0.05;

    double targetOffSetAngle_Vertical = LimelightHelpers.getTY("limelight");

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffSetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distanceFromLimelighToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    double targetingForwardSpeed = distanceFromLimelighToGoalInches * kP;
    targetingForwardSpeed *= DriveConstants.kMaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;



  }

  double[] limelight_align_and_range() {
    // Constants for proportional control
    double kP_Aim = 0.025;   // Aiming proportional gain (tx)
    double kP_Align = 0.02;  // Alignment proportional gain (ty)
    double kP_Range = 0.05;   // Distance proportional gain (ta)

    // Get Limelight data
    double tx = LimelightHelpers.getTX("limelight");  // Horizontal offset
    double ty = LimelightHelpers.getTY("limelight");  // Vertical offset
    double ta = LimelightHelpers.getTA("limelight");  // Target area
    double targetYaw = LimelightHelpers.getBotPose("limelight")[5];
    double currentYaw = m_swerve.getRobotYaw();

    // Calculate angular velocity for aiming
    double aimingRot = tx * kP_Aim * DriveConstants.kMaxAngularSpeed;

    // Desired area (distance goal) - This should be tuned based on real-world measurements
    double desiredArea = 4.5;  // Example: adjust based on desired distance
    double distanceError = desiredArea - ta;

    // Forward/backward speed for maintaining distance
    double forwardSpeed = distanceError * kP_Range * DriveConstants.kMaxSpeed;

    // Strafe speed for aligning parallel
    double strafeSpeed = -tx * kP_Align * DriveConstants.kMaxSpeed;

    // Return an array of values
    return new double[]{forwardSpeed, strafeSpeed, aimingRot};
}

  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.07))
            * DriveConstants.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.07))
            * DriveConstants.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.07))
            * DriveConstants.kMaxAngularSpeed;

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    if(m_controller.getAButton() && LimelightHelpers.getFiducialID("limelight") == 2)
    {
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;
    } 
    // Override manual control when A button is pressed
    if (m_controller.getBButton()&& LimelightHelpers.getFiducialID("limelight") == 21 ) {
      double[] limelightOutputs = limelight_align_and_range();
      xSpeed = limelightOutputs[0]; // Forward/backward
      ySpeed = limelightOutputs[1]; // Sideways alignment
      //rot = limelightOutputs[2];    // Rotation
  }
    /*if(m_controller.getAButton())
    {
    double kPaim = 0.01;
    double kPdistance = 0.05;

    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kPaim;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kPdistance;
      double min_aim_command = 0.05;
      final var heading_error = -targetingAngularVelocity;
      final var distance_error = -targetingForwardSpeed;
      var steering_adjust = 0.0;

      if (targetingAngularVelocity > 1.0) {
        steering_adjust = kPaim * heading_error - min_aim_command;
      } else if (targetingAngularVelocity < -1.0)
      {
        steering_adjust = kPaim * heading_error + min_aim_command;
      }

      final var distance_adjust = kPdistance * distance_error;
      xSpeed += steering_adjust + distance_adjust;

    }*/


    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
   }

    @Override
    public void teleopExit() {

    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {

    }

    @Override
    public void testExit() {
        
    }
}
