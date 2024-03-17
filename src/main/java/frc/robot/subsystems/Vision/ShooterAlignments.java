package frc.robot.subsystems.Vision;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.Speaker;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.DriveConstants;
import frc.robot.subsystems.Drive.DriveConstants.ModuleConstants;
import frc.robot.subsystems.Drive.GyroIO;
import frc.robot.subsystems.Drive.GyroIOInputsAutoLogged;
import frc.robot.subsystems.Drive.GyroIOPigeon2;
import frc.robot.subsystems.Drive.ModuleIO;
import frc.robot.subsystems.Drive.ModuleIOSim;
import frc.robot.subsystems.Drive.ModuleIOSparkMax;
import frc.robot.util.CANSpark;
import frc.robot.util.CANSpark.Motor;

public class ShooterAlignments{
    private InterpolatingTreeMap<Double, Double> interpolateMap = new InterpolatingDoubleTreeMap();
    private Rotation3d AprilTagAngle = new Rotation3d();
    private PhotonVision photonVision;
    private PhotonVisionIO IO;
    private Drive drive;
    private double angle;
    private Rotation3d SpeakerAngle = new Rotation3d();
    private PIDController rotationController = new PIDController(4,0,0);
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  

    
    
    public ShooterAlignments(Drive drive){
       this.drive = drive;
    }


    public void periodic(){
        addValuesToMap();
        setMotors();
        SmartDashboard.putNumber("distance", getDistanceToSpeaker());               
        SmartDashboard.putString("speaker angle", SpeakerAngle.toRotation2d().toString());
        SmartDashboard.putNumber("DriveRot", getRotation());
        SmartDashboard.putNumber("armangle",angleArmToSpeaker());
    }

    public void addValuesToMap() {
        interpolateMap.put(0.0, 0.0);
        interpolateMap.put(100.0, 100.0);
    }
    public void setMotors(){ 
        rotationController.setSetpoint(getRotation());
        rotationController.setTolerance(.05);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        double rotvalue = rotationController.calculate(drive.getPose().getRotation().getRadians());
       
       ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0,0,rotvalue);
       drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
    }
    

       public double getDistanceToSpeaker() {
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

        double distanceX = drive.getPose().getX() - speakerX;
        double distanceY = drive.getPose().getY() - speakerY;
        distance = Math.sqrt(Math.exp(distanceX) + Math.exp(distanceY));
        return distance;
    }

    public double headingToFaceSpeaker() {
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
        double distanceX = drive.getPose().getX() - speakerX;
        double distanceY = drive.getPose().getY() - speakerY;
        double angle = Math.toDegrees(Math.atan2(distanceY, distanceX));
        angle = Math.toRadians((angle + offset + 360) % 360);
        return angle;
    }

    public double angleArmToSpeaker(){
        double distance = getDistanceToSpeaker();
        double armAngle = interpolateMap.get(distance);
        return armAngle;
    }




    public double getRotation(){
        SpeakerAngle = new Rotation3d(0, 0 , headingToFaceSpeaker());
        SpeakerAngle.toRotation2d();
        double targetAngle = (SpeakerAngle.toRotation2d().getRadians());
        


        return targetAngle;
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