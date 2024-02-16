// package frc.robot.subsystems.Vision;

// import org.opencv.photo.Photo;
// import org.photonvision.PhotonCamera;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.Vector;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.numbers.N5;
// import edu.wpi.first.math.numbers.N7;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
// import frc.robot.subsystems.Drive.Drive;



// public class PoseEstimator {
//     private final PhotonCamera photonCamera;
//     private final Drive drive;

//     AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
//     PhotonCamera camera = new PhotonCamera("Left");

//     private final static Vector<N7> stateStdDevs = VecBuilder.fill(.05,.05,Units.degreesToRadians(5),.05,.05,.05,.05);
//     private final static Vector<N5> LocalStdDevs = VecBuilder.fill(Units.degreesToRadians(.01), .01, .01, .01, .01);
//     private final static Vector<N3> VisionMeasurements = VecBuilder.fill(.05, .05, Units.degreesToRadians(5));

//     private final SwerveDrivePoseEstimator<N7, N7, N5> PoseEstimator;
//     private final Field2d field2d = new Field2d();
//     private double previousPipelineTimestamp = 0;

//     public PoseEstimatorSubsystem(PhotonCamera photonCamera, Drive drive, Object PoseEstimator){
//         this.photonCamera = photonCamera;
//         this.drive = drive;

//         ShuffleboardTab tab = Shuffleboard.getTab("Vision");
//         PoseEstimator = new SwerveDrivePoseEstimator<N7, N7, N5>(
//             Nat.N7(), 
//             Nat.N7(),
//             Nat.N5(), 
//             drive.getGyroscopeRotation(),
//             drive.getDrivetrainState())

//     }

// }   
