package org.metuchenmomentum.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        return run(
            () -> shooter.setIndexerSpeed(ShooterConstants.kIndexerInSpeed)
        );
    }

    public Command pivotUp() {
        return new Command() {
            @Override
            public void execute() {
                shooter.turnShooter(ShooterConstants.kShooterPivotUpSpeed);
            }

            @Override
            public void end(boolean interrupted) {
                shooter.turnShooter(0);
            }
        };
    }

    public Command pivotDown() {
        return new Command() {
            @Override
            public void execute() {
                shooter.turnShooter(ShooterConstants.kShooterPivotDownSpeed);
            }

            @Override
            public void end(boolean interrupted) {
                shooter.turnShooter(0);
            }
        };
    }

    public Command turnToAmp() {
        return run(() -> shooter.setShooterPosition(-66));
    }

    public Command turnToHandoff() {
        return run(() -> shooter.setShooterPosition(-23));
    }

    public Command resetPosition() {
        return run(() -> shooter.setShooterPosition(0));
    }

    public void stop() {
        shooter.setShooterSpeed(0);
        shooter.setIndexerSpeed(0);
    }

    public void resetEncoder() {
        shooter.reset();
    }

    @Override
    public void periodic() {
         SmartDashboard.putNumber("Shooter Position", Math.round(shooter.getShooterPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }
}
