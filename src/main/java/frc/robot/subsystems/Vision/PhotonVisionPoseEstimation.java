package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonVisionPoseEstimation implements PhotonVisionIO {
    private PhotonCamera Left;
    private PhotonCamera Right;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final PhotonPoseEstimator LeftPoseEstimator;
    private final PhotonPoseEstimator RightPoseEstimator;
    private EstimatedRobotPose estimatedLeftPose = new EstimatedRobotPose(new Pose3d(), 0, new ArrayList<PhotonTrackedTarget>(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    private EstimatedRobotPose estimatedRightPose = new EstimatedRobotPose(new Pose3d(), 0, new ArrayList<PhotonTrackedTarget>(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    private Transform3d Leftpose = new Transform3d(.16,.26,0, new Rotation3d(Math.toRadians(30),Math.toRadians(72.3),0));
    private Transform3d Rightpose = new Transform3d(.16,-.26,0, new Rotation3d(Math.toRadians(30),Math.toRadians(72.3),0));

    public PhotonVisionPoseEstimation(){
        this.Left = new PhotonCamera("Left");
        this.Right = new PhotonCamera("Right");
        this.LeftPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,Left,Leftpose);
        this.RightPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,Right,Rightpose);
    }
    
        public void updateInputs(PhotonVisionIOInputsAutoLogged inputs) {
        double LeftAmbiguitySum = 0;
        double RightAmbiguitySum = 0;

        inputs.RightConfidence = 0;
        inputs.LeftConfidence = 0;

        Optional<EstimatedRobotPose> LeftPoseOptional = LeftPoseEstimator.update();
        if (LeftPoseOptional.isPresent()) {
            estimatedLeftPose = LeftPoseOptional.get();
            var leftVisionPoseUpdate = LeftPoseOptional;
            var leftVisionPose = leftVisionPoseUpdate.get().estimatedPose;
            inputs.estimatedLeftPose = estimatedLeftPose.estimatedPose;
            inputs.estimatedLeftPoseTimestamp = estimatedLeftPose.timestampSeconds;
            SmartDashboard.putNumber("Left Vision Pose X", leftVisionPose.getX());
            SmartDashboard.putNumber("Left Vision Pose Y", leftVisionPose.getY());
            var LeftTargetsSeen = estimatedLeftPose.targetsUsed.size();
            inputs.visibleLeftFiducialIDs = new int[LeftTargetsSeen];

            for (int i = 0; i < LeftTargetsSeen; i++) {
                var target = estimatedLeftPose.targetsUsed.get(i);
                inputs.visibleLeftFiducialIDs[i] = target.getFiducialId();
                LeftAmbiguitySum += target.getPoseAmbiguity();
            }   

            inputs.LeftConfidence = 1 - (LeftAmbiguitySum / inputs.visibleLeftFiducialIDs.length);
        }
        Optional<EstimatedRobotPose> RightPoseOptional = RightPoseEstimator.update();
        
        if (RightPoseOptional.isPresent()) {
            estimatedRightPose = RightPoseOptional.get();

            inputs.estimatedRightPose = estimatedRightPose.estimatedPose;
            inputs.estimatedRightPoseTimestamp = estimatedRightPose.timestampSeconds;
        
            var RightTargetsSeen = estimatedRightPose.targetsUsed.size();
            inputs.visibleRightFiducialIDs = new int[RightTargetsSeen];
        
       
            for (int i = 0; i < RightTargetsSeen; i++) {
                var target = estimatedRightPose.targetsUsed.get(i);
                inputs.visibleRightFiducialIDs[i] = target.getFiducialId();
                RightAmbiguitySum += target.getPoseAmbiguity();
            }  

            inputs.RightConfidence = 1 - (RightAmbiguitySum / inputs.visibleRightFiducialIDs.length); 
        } 
    }
    public List<PhotonTrackedTarget> getLeftTrackedTargets() {
        return estimatedLeftPose.targetsUsed;
    }

    public List<PhotonTrackedTarget> getRightTrackedTargets() {
        return estimatedRightPose.targetsUsed;
    }

    @AutoLogOutput
    public Pose3d[] getLeftTagPoses() {
        var LeftTargets = estimatedLeftPose.targetsUsed;
        Pose3d[] LeftTagPoses = new Pose3d[LeftTargets.size()];
        for(int i = 0; i < LeftTagPoses.length; i++) {
            LeftTagPoses[i] = aprilTagFieldLayout.getTagPose(LeftTargets.get(i).getFiducialId()).get();
        }
        return LeftTagPoses;
    }

    @AutoLogOutput
    public Pose3d[] getRightTagPoses() {
        var RightTargets = estimatedRightPose.targetsUsed;
        Pose3d[] RightTagPoses = new Pose3d[RightTargets.size()];
        for(int i = 0; i < RightTagPoses.length; i++) {
            RightTagPoses[i] = aprilTagFieldLayout.getTagPose(RightTargets.get(i).getFiducialId()).get();
        }
        return RightTagPoses;
    }
    
    
}