package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import frc.robot.FieldConstants.Speaker;
import frc.robot.subsystems.Drive.Drive;

public class ShooterAlignments{
    private InterpolatingTreeMap<Double, Double> interpolateMap = new InterpolatingDoubleTreeMap();
    private Rotation2d AprilTagAngle = new Rotation2d();
    private PhotonVision photonVision;
    private PhotonVisionIO IO;
    private Drive drive;
    private double angle;
    private double distance = drive.getDistanceToSpeaker();
    private double armAngle = interpolateMap.get(distance);
    private Rotation3d SpeakerAngle = new Rotation3d();


    public void periodic(){
addValuesToMap();
}

    public void addValuesToMap() {
        interpolateMap.put(0.0, 9.5);
        interpolateMap.put(10.0, 9.5);
    }
    
    public ShooterAlignments(Drive drive){
        this.drive = drive; 
    }

    public double angleArmToSpeaker(){
        double distance = drive.getDistanceToSpeaker();
       double armAngle = interpolateMap.get(distance);
       return armAngle;
    }

    public Rotation3d faceSpeaker(){
        double angle = drive.headingToFaceSpeaker();
        SpeakerAngle = new Rotation3d(0, 0 , angle);
        return SpeakerAngle;
    }

    public double getRotation(){
        Pose2d Opose = drive.getPose();
        SpeakerAngle.toRotation2d();
        double targetAngle = (SpeakerAngle.toRotation2d().getDegrees() + 360) % 360;
        
        Rotation2d RobotRotation = Opose.getRotation();
        double robotAngle = (RobotRotation.getDegrees() + 360) % 360;
        double distanceToSpeaker;

        double a  = targetAngle - robotAngle;
        double b = targetAngle - robotAngle + 360;
        double c = targetAngle - robotAngle -  360;

        if (Math.abs(a) < Math.abs(b) && Math.abs(a) < Math.abs(c)){
            distanceToSpeaker = a;
        } else if (Math.abs(b) < Math.abs(c)) {
            distanceToSpeaker = b;
        } else {
            distanceToSpeaker = c;
        }
        
        double output = Math.cbrt(distanceToSpeaker);
        if (output > 1) {
            output = 1;
        } else if (output < -1.0) {
            output = -1;
        }
        

        return output;
}





}