package org.metuchenmomentum.robot.subsystems.intake;

public interface IntakeIO {
    /**
     * Sets the speed of the intake.
     * @param speed The speed to set the intake to.
     */
    public void setIntakeSpeed(double speed);

    /**
     * Sets the speed of the pivot.
     * @param speed The speed to set the pivot to.
     */
    public void setPivotSpeed(double speed);

    /**
     * Sets the pivot to a specified posiiton.
     * @param target The position to set the pivot to.
     */
    public void setPivotPosition(double setpoint);

    /**
     * Gets the position of the pivot.
     * @return The position of the pivot.
     */
    public double getPivotPosition();

    public void setP(double p);

    public void setI(double i);

    public void setD(double d);

    public void setFF(double ff);

    public double getP();

    public double getFF();

    public double getI();

    public double getD();

    public void reset();
}
