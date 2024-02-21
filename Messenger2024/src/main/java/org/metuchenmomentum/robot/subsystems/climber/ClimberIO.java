package org.metuchenmomentum.robot.subsystems.climber;

public interface ClimberIO {
    /** 
     * Sets the speed of the left hook.
     * @param speed The speed to set the left hook to.
     */
    public void setLeftHookSpeed(double speed);

    /**
     * Sets the speed of the right hook.
     * @param speed The speed to set the right hook to.
     */
    public void setRightHookSpeed(double speed);

    /**
     * Sets the speed of both hooks.
     * @param speed The speed to set the hooks to.
     */
    public void setHookSpeeds(double speed);

    /**
     * Returns the height of the left hook.
     * @return The height of the left hook.
     */
    public double getLeftHookHeight();

    /**
     * Returns the height of the right hook.
     * @return The height of the right hook.
     */
    public double getRightHookHeight();
}
