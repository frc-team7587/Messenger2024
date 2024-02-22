package org.metuchenmomentum.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooter;

    public Shooter(ShooterIO shooter) {
        this.shooter = shooter;
    }

    public Command prepareLaunch() {
        return run(() -> shooter.setShooterSpeed(1));
    }

    public Command launchNote() {
        return run(() -> shooter.setIndexerSpeed((1)));
    }
}
