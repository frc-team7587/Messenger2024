package org.metuchenmomentum.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO shooter;

    public Shooter(ShooterIO shooter) {
        this.shooter = shooter;
    }

    public Command prepareSpeaker() {
        return run(() -> shooter.setShooterSpeed(ShooterConstants.kSpeakerShootingSpeed));
    }
    public Command prepareSpeakerPosition() {
        return run(() -> shooter.setShooterPosition(-20));
    }

    public Command prepareAmplify() {
        return run(() -> shooter.setShooterSpeed(ShooterConstants.kAmpShootingSpeed));
    }

    public Command launchNote() {
        return run(
            () -> shooter.setIndexerSpeed(ShooterConstants.kIndexerInSpeed)
        );
    }
    public Command manualShoot() {
        return new SequentialCommandGroup(
             prepareSpeaker()
            .withTimeout(.5)
            .andThen(launchNote().withTimeout(1))   
        );
    }
    public Command clearShootingWheels() {
        return takeBackALittleBitShooter();
    }
    
    public Command takeBackALittleBitShooter() {
        return run(() -> shooter.setShooterSpeed(-0.3));
   
    }

    public Command takeBackALittleBitIndexer() {
        return run(() -> shooter.setIndexerSpeed(0.1));
    }

    public Command loadNote() {
        return startEnd(
            () -> shooter.setIndexerSpeed(-1),
            () -> shooter.setIndexerSpeed(0)
        );
    }

    public Command amplify() {
        return new SequentialCommandGroup(
            turnToAmp().withTimeout(1),
            prepareAmplify().withTimeout(0),
            launchNote().withTimeout(1),
            stopShooter().withTimeout(0),
            stopIndexer().withTimeout(0),
            resetPosition()
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
        return run(() -> shooter.setShooterPosition(-70)); //comp -70
    }

    public Command turnToHandoff() {
        return run(() -> shooter.setShooterPosition(-28));
    }

    public Command resetPosition() {
        return run(() -> shooter.setShooterPosition(0));
    }

    public Command stopShooter() {
        return run(
            () -> shooter.setShooterSpeed(0)
        );
    }

     public Command stopIndexer() {
        return run(
            () -> shooter.setIndexerSpeed(0)
        );
    }


    public void resetEncoder() {
        shooter.reset();
    }

    @Override
    public void periodic() {
         SmartDashboard.putNumber("Shooter Position", Math.round(shooter.getShooterPosition() * Math.pow(10, 2)) / Math.pow(10, 2));
    }
}
