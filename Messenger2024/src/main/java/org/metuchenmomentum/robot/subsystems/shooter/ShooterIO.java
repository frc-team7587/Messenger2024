package org.metuchenmomentum.robot.subsystems.shooter;

public interface ShooterIO {
    /**
     * Sets the speed of the indexer.
     * @param speed The speed to set the indexer to.
     */
    public void setIndexerSpeed(double speed);

    /**
     * Sets the speed of the shooter.
     * @param speed The speed to set the shooter to.
     */
    public void setShooterSpeed(double speed);

    /**
     * Sets the position of the shooter.
     * @param target The position to set the shooter to.
     */
    public void setShooterPosition(double target);

    /**
     * Gets the position of the shooter.
     * @return The position of the shooter.
     */
    public double getShooterPosition();
    
    public void turnShooter(double speed);

    public void setP(double p);

    public void setI(double i);

    public void setD(double d);

    public double getP();

    public double getI();

    public double getD();

    public void reset();
}
