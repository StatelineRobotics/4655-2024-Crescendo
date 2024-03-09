package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose3d;

 public interface PhotonVisionIO {
    @AutoLog
    public static class PhotonVisionIOInputs {
        public Pose3d estimatedLeftPose = new Pose3d();
        public Pose3d estimatedRightPose = new Pose3d();
        public double estimatedLeftPoseTimestamp = 0.0;
        public double estimatedRightPoseTimestamp = 0.0;
        public int[] visibleLeftFiducialIDs = new int[]{};
        public int[] visibleRightFiducialIDs = new int[]{};
        public double LeftConfidence = 0.0;
        public double RightConfidence = 0.0;
    }

    public default List<PhotonTrackedTarget> getFrontTrackedTargets() {
        return new ArrayList<>();
    }

    public default List<PhotonTrackedTarget> getRearTrackedTargets() {
        return new ArrayList<>();
    }

    public default void updateInputs(PhotonVisionIOInputsAutoLogged inputs) {
        
    }
}   

