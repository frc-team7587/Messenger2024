package org.metuchenmomentum.robot.subsystems.intake;

public class IntakeConstants {
    public static final double kIntakeInSpeed = 1.0;
    public static final double kIntakeOutSpeed = -1.0;
    public static final double kPivotUpSpeed = -0.5;
    public static final double kPivotDownSpeed = 0.5;

    public static final double kIntakeGroundPosition = -39.0;
    public static final double kIntakeShooterPosition = -0.5;
    public static final double kIntakeNeutralPosition = -15.0;

    public static final int kIntakeMotorID = 9;
    public static final int kIntakePivotMotorID = 10;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.7;
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;
}
