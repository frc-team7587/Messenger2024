package org.metuchenmomentum.robot.subsystems.vision;


// Import necessary libraries
import com.pathplanner.lib.path.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.pathplanner.lib.controllers.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.lang.runtime.SwitchBootstraps;

import org.metuchenmomentum.robot.Constants.DriveConstants;
import org.metuchenmomentum.robot.subsystems.drive.SwerveDrive;
import org.metuchenmomentum.robot.subsystems.vision.LimelightHelpers;

public class AlignToAprilTagCommand extends Command {
    private final SwerveDrive swerveDrive;
    private final double kP = 0.015;

    public AlignToAprilTagCommand(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double tx = LimelightHelpers.getTX("limelight");
        double rot = -tx * kP;
        swerveDrive.drive(0, 0, rot, true, 0.02);
    }

    @Override
    public boolean isFinished() {
        double tx = LimelightHelpers.getTX("limelight");
        return Math.abs(tx) < 1.0;
    }
}

