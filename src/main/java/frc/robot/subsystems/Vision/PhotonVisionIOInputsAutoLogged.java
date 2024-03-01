package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PhotonVisionIOInputsAutoLogged extends PhotonVisionIO.PhotonVisionIOInputs implements LoggableInputs, Cloneable {
    @Override
  public void toLog(LogTable table) {
    table.put("EstimatedFrontPose", estimatedLeftPose);
    table.put("EstimatedRearPose", estimatedRightPose);
    table.put("EstimatedFrontPoseTimestamp", estimatedFrontPoseTimestamp);
    table.put("EstimatedRearPoseTimestamp", estimatedRearPoseTimestamp);
    table.put("VisibleFrontFiducialIDs", visibleFrontFiducialIDs);
    table.put("VisibleRearFiducialIDs", visibleRearFiducialIDs);
    table.put("FrontConfidence", frontConfidence);
    table.put("RearConfidence", rearConfidence);
  }

  @Override
  public void fromLog(LogTable table) {
    estimatedLeftPose = table.get("EstimatedFrontPose", estimatedLeftPose);
    estimatedRightPose = table.get("EstimatedRearPose", estimatedRightPose);
    estimatedFrontPoseTimestamp = table.get("EstimatedFrontPoseTimestamp", estimatedFrontPoseTimestamp);
    estimatedRearPoseTimestamp = table.get("EstimatedRearPoseTimestamp", estimatedRearPoseTimestamp);
    visibleFrontFiducialIDs = table.get("VisibleFrontFiducialIDs", visibleFrontFiducialIDs);
    visibleRearFiducialIDs = table.get("VisibleRearFiducialIDs", visibleRearFiducialIDs);
    frontConfidence = table.get("FrontConfidence", frontConfidence);
    rearConfidence = table.get("RearConfidence", rearConfidence);
  }

  public PhotonVisionIOInputsAutoLogged clone() {
    PhotonVisionIOInputsAutoLogged copy = new PhotonVisionIOInputsAutoLogged();
    copy.estimatedLeftPose = this.estimatedLeftPose;
    copy.estimatedRightPose = this.estimatedRightPose;
    copy.estimatedFrontPoseTimestamp = this.estimatedFrontPoseTimestamp;
    copy.estimatedRearPoseTimestamp = this.estimatedRearPoseTimestamp;
    copy.visibleFrontFiducialIDs = this.visibleFrontFiducialIDs.clone();
    copy.visibleRearFiducialIDs = this.visibleRearFiducialIDs.clone();
    copy.frontConfidence = this.frontConfidence;
    copy.rearConfidence = this.rearConfidence;
    return copy;
  }
}


