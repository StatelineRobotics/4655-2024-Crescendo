package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;

public interface PhotonVisionIO {
    @AutoLog
    public static class PhotonVisionIOInputs {
        public Pose3d estimatedFrontPose = new Pose3d();
        public Pose3d estimatedRearPose = new Pose3d();
        public double estimatedFrontPoseTimestamp = 0.0;
        public double estimatedRearPoseTimestamp = 0.0;
        public int[] visibleFrontFiducialIDs = new int[]{};
        public int[] visibleRearFiducialIDs = new int[]{};
        public double frontConfidence = 0.0;
        public double rearConfidence = 0.0;
    }

    public default List<PhotonTrackedTarget> getFrontTrackedTargets() {
        return new ArrayList<>();
    }

    public default List<PhotonTrackedTarget> getRearTrackedTargets() {
        return new ArrayList<>();
    }

    public default void updateInputs(PhotonVisionIOInputs inputs) {}
}