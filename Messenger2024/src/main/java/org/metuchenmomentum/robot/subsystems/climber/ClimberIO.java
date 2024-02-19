package org.metuchenmomentum.robot.subsystems.climber;

public interface ClimberIO {
    /** Raises the left hook. */
    public void raiseLeftHook();

    /** Raises the right hook. */
    public void raiseRightHook();

    /** Lowers the left hook. */
    public void lowerLeftHook();

    /** Lowers the right hook. */
    public void lowerRightHook();

    /** Raises the hooks. */
    public void raiseHooks();

    /** Lowers the hooks. */
    public void lowerHooks();

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
