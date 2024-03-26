package frc.robot.subsystems.Vision;

import java.util.Optional;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.DriveConstants;


public class ShooterAlignments{
    private InterpolatingTreeMap<Double, Double> interpolateMap = new InterpolatingDoubleTreeMap();
    private PhotonVision photonVision;
    private Drive drive;
    private Rotation3d SpeakerAngle = new Rotation3d();
    private PIDController rotationController = new PIDController(4,0,0);

    public double armAngle;
    public boolean VhasTarget;

  

    
    
    public ShooterAlignments(Drive drive, PhotonVision photonVision){
       this.drive = drive;
       this.photonVision = photonVision;
    }


    public void periodic(){
        addValuesToMap();
        Optional<Pose2d> estimatedPose = photonVision.getEstimatedPose();
        if (estimatedPose.isPresent() && photonVision.getPoseAmbiguity()) {
            VhasTarget = true;
            Pose2d Vpose = estimatedPose.get();
            double distanceToSpeaker = getDistanceToSpeaker(Vpose);
            armAngle = angleArmToSpeaker(distanceToSpeaker);
            SmartDashboard.putNumber("armangle", armAngle);
            setMotors(Vpose);

        } else {
            VhasTarget = false;
            armAngle = 22.5;

        }


    }

    public void addValuesToMap() {
        interpolateMap.put(1.82, 6.8);
        interpolateMap.put(2.9, 22.5);
    }

    public void setMotors(Pose2d Vpose){ 
        rotationController.setSetpoint(headingToFaceSpeaker(Vpose));
        rotationController.setTolerance(.05);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        double rotvalue = rotationController.calculate(drive.getPose().getRotation().getRadians());
       
       ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,rotvalue);
       drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }
    

       public double getDistanceToSpeaker(Pose2d Vpose) {
        double distance;
        Translation2d blueSpeaker = FieldConstants.Speaker.bluecenterSpeakerOpening;
        Translation2d redSpeaker = FieldConstants.Speaker.redcenterSpeakerOpening;
        double speakerX;
        double speakerY;        
        if (DriverStation.getAlliance().get() == Alliance.Blue){
            speakerX = blueSpeaker.getX();
            speakerY = blueSpeaker.getY();
        }
        else{
            speakerX = redSpeaker.getX();
            speakerY = redSpeaker.getY();
        }

        double distanceX = Vpose.getX() - speakerX;
        double distanceY = Vpose.getY() - speakerY;
        distance = Math.sqrt(Math.exp(distanceX) + Math.exp(distanceY));
        return distance;
    }

    public double headingToFaceSpeaker(Pose2d Vpose) {
        Translation2d blueSpeaker = FieldConstants.Speaker.bluecenterSpeakerOpening;
        Translation2d redSpeaker = FieldConstants.Speaker.redcenterSpeakerOpening;
        double speakerX;
        double speakerY;
        double offset = 0;
        if (DriverStation.getAlliance().get() == Alliance.Blue){
            speakerX = blueSpeaker.getX();
            speakerY = blueSpeaker.getY();
            offset = 0;
        }
        else{
            speakerX = redSpeaker.getX();
            speakerY = redSpeaker.getY();
            offset = 0;
        }
        double distanceX = Vpose.getX() - speakerX;
        double distanceY = Vpose.getY() - speakerY;
        double angle = Math.toDegrees(Math.atan2(distanceY, distanceX));
        angle = Math.toRadians((angle + offset + 360) % 360);

        SpeakerAngle = new Rotation3d(0, 0 , angle);
        double targetAngle = SpeakerAngle.toRotation2d().getRadians();

        return targetAngle;
    }

    public double angleArmToSpeaker(double distance){
        double armAngle = interpolateMap.get(distance);
        return armAngle;
    }






/* 
public void visionAlignment(){
    if (photonVision.getLatestResult().hasTargets()){
    AprilTagAngle = new Rotation3d(0,0,photonVision.getYaw());
    Rotation2d RobotAngle = drive.getPose().getRotation();

    }
}
*/


}