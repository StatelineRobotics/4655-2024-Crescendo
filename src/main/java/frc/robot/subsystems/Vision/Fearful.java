package frc.robot.subsystems.Vision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.Constants.OIConstants;


public class Fearful extends Command {
    

    private static final Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3,3);
    private static final Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3,3);
    private static final Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(10,10);

    private static final int Tag_to_chase = 9;

    private static final Transform3d Tag_to_goal = new Transform3d(
        new Translation3d(1,0,0),
        new Rotation3d(0,0,Math.PI)
        );

    private PhotonCamera Left;
    private Drive drive;
    private Supplier<Pose2d> poseProvider;
    
    private final ProfiledPIDController xController = new ProfiledPIDController(.04, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(.04, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

    private PhotonTrackedTarget lastTarget;

    public Fearful() {
        Drive drive;
        PhotonCamera Left;
        Supplier<Pose2d> poseProvider;
        this.drive = drive;
        this.Left = Left;
        this.poseProvider = poseProvider;
        xController.setTolerance(.3);
        yController.setTolerance(.3);
        omegaController.setTolerance(Units.degreesToRadians(3));

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        var Robotpose = poseProvider.get();
        xController.reset(Robotpose.getX());
        yController.reset(Robotpose.getY());

    }

    @Override
    public void Execute(){
        Robotpose
    }
    
}