package frc.robot.subsystems.Vision;


import java.util.function.Supplier;

import static frc.robot.Constants.AutoConstants.THETA_kD;
import static frc.robot.Constants.AutoConstants.THETA_kI;
import static frc.robot.Constants.AutoConstants.THETA_kP;
import static frc.robot.Constants.AutoConstants.X_kD;
import static frc.robot.Constants.AutoConstants.X_kI;
import static frc.robot.Constants.AutoConstants.X_kP;
import static frc.robot.Constants.AutoConstants.Y_kD;
import static frc.robot.Constants.AutoConstants.Y_kI;
import static frc.robot.Constants.AutoConstants.Y_kP;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;

/**
 * Pose estimator that uses odometry and AprilTags with PhotonVision.
 */
public class PoseEstimatorSubsystem extends SubsystemBase {
  private static final double TRANSLATION_TOLERANCE = 0.02;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(1.5, 1.5, 1.5);

  private final Supplier<Rotation2d> rotationSupplier;
  private final Supplier<SwerveModulePosition[]> modulePositionSupplier;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  private final PhotonRunnable photonEstimator = new PhotonRunnable();
  private final Notifier photonNotifier = new Notifier(photonEstimator);
  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private boolean sawTag = false;
  private Pose2d goalPose;
  private Supplier<Pose2d> poseProvider;

  public PoseEstimatorSubsystem(
      Supplier<Rotation2d> rotationSupplier,
       Supplier<SwerveModulePosition[]> modulePositionSupplier, TrapezoidProfile.Constraints xyConstraints,
       TrapezoidProfile.Constraints omegaConstraints, Supplier<Pose2d> poseProvider,
       Pose2d goalPose) {
    this.rotationSupplier = rotationSupplier;
    this.modulePositionSupplier = modulePositionSupplier;
    this.poseProvider = poseProvider;
    this.goalPose = goalPose;
    xController = new ProfiledPIDController(X_kP, X_kI, X_kD, xyConstraints);
    yController = new ProfiledPIDController(Y_kP, Y_kI, Y_kD, xyConstraints);
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, omegaConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);   
    
    poseEstimator =  new SwerveDrivePoseEstimator(
        DrivetrainConstants.KINEMATICS,
        rotationSupplier.get(),
        modulePositionSupplier.get(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    // Start PhotonVision thread
    photonNotifier.setName("PhotonRunnable");
    photonNotifier.startPeriodic(0.02);
  }

  public void addDashboardWidgets(ShuffleboardTab tab) {
    tab.add("Field", field2d).withPosition(0, 0).withSize(6, 4);
    tab.addString("Pose", this::getFomattedPose).withPosition(6, 2).withSize(2, 1);
  }

  /**
   * Sets the alliance. This is used to configure the origin of the AprilTag map
   * @param alliance alliance
   */
 
  @Override
  public void periodic() {
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());

    var visionPose = photonEstimator.grabLatestEstimatedPose();
    if (visionPose != null) {
      // New pose from vision
      sawTag = true;
      var pose2d = visionPose.estimatedPose.toPose2d();
    }

    }
    // Set the pose on the dashboard


  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.3f, %.3f) %.2f degrees", 
        pose.getX(), 
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(rotationSupplier.get(), modulePositionSupplier.get(), newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  /**
   * Transforms a pose to the opposite alliance's coordinate system. (0,0) is always on the right corner of your
   * alliance wall, so for 2023, the field elements are at different coordinates for each alliance.
   * @param poseToFlip pose to transform to the other alliance
   * @return pose relative to the other alliance's coordinate system
   */
  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(VisionConstants.FLIPPING_POSE);
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
