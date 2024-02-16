package frc.robot.subsystems.Vision;



import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator photonPoseEstimator;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
  private final PhotonCamera left = new PhotonCamera("Left");
  public PhotonRunnable() {
    PhotonPoseEstimator photonPoseEstimator = null;
    var layout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // PV estimates will always be blue, they'll get flipped by robot thread
    if (left != null) {
      photonPoseEstimator = new PhotonPoseEstimator(
          layout, PoseStrategy.LOWEST_AMBIGUITY, left, APRILTAG_CAMERA_TO_ROBOT.inverse());
    }
    this.photonPoseEstimator = photonPoseEstimator;
  }

  @Override
  public void run() {      
    // Get AprilTag data
    if (photonPoseEstimator != null && left != null) {
      var photonResults = left.getLatestResult();
      SmartDashboard.getBoolean("Photon Has Targets", photonResults.hasTargets());
      if (photonResults.hasTargets() ) {
        photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field
          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= 16.54175
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= 8.0137) {
            atomicEstimatedRobotPose.set(estimatedRobotPose);
          }
        });
      }
    }  
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If it returns a non-null value, it is a
   * new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance. It must be flipped if the current alliance is RED.
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

}
