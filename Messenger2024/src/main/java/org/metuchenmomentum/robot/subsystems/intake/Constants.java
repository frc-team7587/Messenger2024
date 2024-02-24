package org.metuchenmomentum.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int kMotorPort = 0;
    public static final int kEncoderAChannel = 0;
    public static final int kEncoderBChannel = 1;
    public static final int kJoystickPort = 0;

    public static final String kArmPositionKey = "ArmPosition";
    public static final String kArmPKey = "ArmP";

    // The P gain for the PID controller that drives this arm.
    public static final double kDefaultArmKp = 50.0;
    public static final double kDefaultArmSetpointDegrees = 75.0;

    // distance per pulse = (angle per revolution) / (pulses per revolution)
    // = (2 * PI rads) / (4096 pulses)
    public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;

    public static final double kArmReduction = 200;
    public static final double kArmMass = 8.0; // Kilograms
    public static final double kArmLength = Units.inchesToMeters(30);
    public static final double kMinAngleRads = Units.degreesToRadians(-75);
    public static final double kMaxAngleRads = Units.degreesToRadians(255);
}
