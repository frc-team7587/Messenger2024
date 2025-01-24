package org.metuchenmomentum.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    // private final PhotonCamera camera;
    // private PhotonPipelineResult currentResult;
    // private List<PhotonTrackedTarget> currentTargets;
    // private PhotonTrackedTarget currentBestTarget;
    
    // public Vision() {
    //     camera = new PhotonCamera("7587_Camera");
    // }

    // public void getLatestResult() {
    //     currentResult = camera.getLatestResult();
    // }

    // public void getLatestTargets() {
    //     currentTargets = currentResult.getTargets();
    //     currentBestTarget = currentResult.getBestTarget();
    // }

    // public boolean hasTargets() {
    //     return currentResult.hasTargets();
    // }

    // @Override
    // public void periodic() {
    //     SmartDashboard.putBoolean("Has Targets", hasTargets());
    //     if (hasTargets()) {
    //         SmartDashboard.putNumber("Best Target Area", currentBestTarget.getArea());
    //         SmartDashboard.putNumber("Best Target X", currentBestTarget.getPitch());
    //         SmartDashboard.putNumber("Best Target Y", currentBestTarget.getYaw());
    //         SmartDashboard.putNumber("Best Target ID", currentBestTarget.getFiducialId());
    //     }
    // }
}
