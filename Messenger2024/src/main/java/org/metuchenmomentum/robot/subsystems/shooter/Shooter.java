package org.metuchenmomentum.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooter;

    public Shooter(ShooterIO shooter) {
        this.shooter = shooter;
    }

    public Command prepareSpeaker() {
        return run(() -> shooter.setShooterSpeed(ShooterConstants.kSpeakerShootingSpeed));
    }

    public Command prepareAmplify() {
        return run(() -> shooter.setShooterSpeed(ShooterConstants.kAmpShootingSpeed));
    }

    public Command launchNote() {
        return run(() -> shooter.setIndexerSpeed(ShooterConstants.kIndexerInSpeed));
    }

    public Command pivotUp() {
        return run(() -> shooter.turnShooter(ShooterConstants.kShooterPivotUpSpeed));
    }

    public Command pivotDown() {
        return run(() -> shooter.turnShooter(ShooterConstants.kShooterPivotDownSpeed));
    }

    public Command turnToAmp() {
        return run(() -> shooter.setShooterPosition(ShooterConstants.kAmpPosition));
    }

    public Command turnToHandoff() {
        return run(() -> shooter.setShooterPosition(ShooterConstants.kHandoffPosition));
    }

    public Command reset() {
        return run(() -> shooter.setShooterPosition(ShooterConstants.kNeutralPosition));
    }
}
