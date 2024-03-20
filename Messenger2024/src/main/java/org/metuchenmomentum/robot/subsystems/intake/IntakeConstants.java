package org.metuchenmomentum.robot.subsystems.intake;

public class IntakeConstants {
    public static final double kIntakeInSpeed = 0.65;
    public static final double kIntakeOutSpeed = -0.65;
    public static final double kPivotUpSpeed = -0.25;
    public static final double kPivotDownSpeed = 0.25;

    public static final double kIntakeGroundPosition = -36; //comp -35
    public static final double kIntakeShooterPosition = -0.5;
    public static final double kIntakeNeutralPosition = -15.0;

    public static final int kIntakeMotorID = 9;
    public static final int kIntakePivotMotorID = 10;

    public static final double kP = 0.03;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;
    public static final double kMinOutput = -1.0;
    public static final double kMaxOutput = 1.0;

    public static int state =0;
}
