package org.metuchenmomentum.robot.subsystems.shooter;

public class ShooterConstants {
    public static final double kSpeakerShootingSpeed = 1;
    public static final double kAmpShootingSpeed = 0.25;
    public static final double kIndexerInSpeed = -1;
    public static final double kIndexerOutSpeed = 0.5;
    public static final double kShooterInSpeed = 0.5;
    public static final double kShooterOutSpeed = -0.5;
    public static final double kShooterPivotDownSpeed = 0.05;
    public static final double kShooterPivotUpSpeed = -0.1;

    public static final double kNeutralPosition = 0;
    public static final double kHandoffPosition = 16;
    public static final double kAmpPosition = 135;

    public static final int kShootingMotorID = 11;
    public static final int kIndexingMotorID = 12;
    public static final int kShooterPivotMotorID = 13;

    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    public static final double kFF = 0.0;
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
}
