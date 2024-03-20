package org.metuchenmomentum.robot.subsystems.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private PhotonPipelineResult currentResult;
    private PhotonTrackedTarget currentBestTarget;

    private Transform3d kRobotToCam = new Transform3d(
        Units.inchesToMeters(14),
        0,
        Units.inchesToMeters(21),
        new Rotation3d(0, Units.degreesToRadians(-85), 0));
    
    public Vision() {
        camera = new PhotonCamera("7587_Camera");
        estimator = new PhotonPoseEstimator(AprilTagFields.kDefaultField.loadAprilTagLayoutField(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void getLatestResult() {
        currentResult = camera.getLatestResult();
    }

    public Optional<EstimatedRobotPose> getLatestPose(Pose2d currentEstimate) {
        estimator.setReferencePose(currentEstimate);
        Optional<EstimatedRobotPose> pose = estimator.update(currentResult);
        return pose;
    }

    public void getLatestTargets() {
        currentBestTarget = currentResult.getBestTarget();
    }

    public boolean hasTargets() {
        return currentResult.hasTargets();
    }

    public double getTimestamp() {
        return currentResult.getTimestampSeconds();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Targets", hasTargets());
        if (hasTargets()) {
            SmartDashboard.putNumber("Best Target Area", currentBestTarget.getArea());
            SmartDashboard.putNumber("Best Target Pitch", currentBestTarget.getPitch());
            SmartDashboard.putNumber("Best Target Yaw", currentBestTarget.getYaw());
            SmartDashboard.putNumber("Best Target ID", currentBestTarget.getFiducialId());
        }
    }
}
