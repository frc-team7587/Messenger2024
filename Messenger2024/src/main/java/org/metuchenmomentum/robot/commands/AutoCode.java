package org.metuchenmomentum.robot.commands;

import org.metuchenmomentum.robot.subsystems.intake.Intake;
import org.metuchenmomentum.robot.subsystems.intake.IntakeSparkMax;
import org.metuchenmomentum.robot.subsystems.shooter.Shooter;
import org.metuchenmomentum.robot.subsystems.shooter.ShooterSparkMax;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCode extends SequentialCommandGroup {
    private Shooter shooter = new Shooter(new ShooterSparkMax());
    private Intake intake = new Intake(new IntakeSparkMax());

    public AutoCode() {
        addCommands(
            shooter.prepareSpeakerPosition().withTimeout(.1),
            shooter.prepareSpeaker().withTimeout(.5),
            shooter.launchNote().alongWith(intake.intakeOut()).withTimeout(1),
            shooter.stopShooter().withTimeout(.1),
            intake.stopIntake().withTimeout(0).alongWith(shooter.stopIndexer())
        );
    }
}