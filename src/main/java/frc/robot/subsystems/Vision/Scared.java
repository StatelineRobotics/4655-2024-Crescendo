// package frc.robot.subsystems.Vision;

// import java.util.function.Supplier;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.Drive.Drive;
// import frc.robot.Constants.OIConstants;

// public class Scared extends Command {


//     private static final Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3,3);
//     private static final Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3,3);
//     private static final Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(10,10);

//     private static final int Tag_To_chase = 9;
//     private static final Transform3d Tag_To_Goal =
//         new Transform3d(
//             new Translation3d(1.5,0.0,0.0),
//             new Rotation3d(0,0,Math.PI));
//     private static final Transform3d Robot_To_Target =
//         new Transform3d(
//             new Translation3d(1.5,0.0,0.0),
//             new Rotation3d(0,0,Math.PI));
//     private PhotonCamera left;
//     private Drive drive;
//     private Supplier<Pose2d> poseProvider;

//     private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
//     private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
//     private final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);
    
//     private PhotonTrackedTarget lastTarget;

//     public void ScaredPremium(
//             PhotonCamera left,
//             Drive drive,
//             Supplier<Pose2d> poseProvider) {

//         this.drive = drive;
//         this.poseProvider = poseProvider;

//         xController.setTolerance(.2);
//         yController.setTolerance(.2);
//         omegaController.setTolerance(Units.degreesToRadians(3));
//         omegaController.enableContinuousInput(-Math.PI, Math.PI);

//         addRequirements(drive);
//     }

    
//     @Override
//     public void initialize() {
//         lastTarget = null;
//         var robotPose = poseProvider.get();
//         omegaController.reset(robotPose.getRotation().getRadians());
//         xController.reset(robotPose.getX());
//         yController.reset(robotPose.getY());
//     }

//     public boolean atGoal() {
//         return xController.atGoal() && yController.atGoal() && omegaController.atGoal();
//       }

//     @Override
//     public void execute() {
//         var robotPose2d = poseProvider.get();
//         var robotPose = 
//             new Pose3d(
//                 robotPose2d.getX(),
//                 robotPose2d.getY(),
//                 0.0,
//                 new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));

//         var photonRes = left.getLatestResult();
//         if (photonRes.hasTargets()) {
//             var targetOpt = photonRes.getTargets().stream()
//                 .filter(t -> t.getFiducialId() == Tag_To_chase)
//                 .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() > -1)
//                 .findFirst();
//             if (targetOpt.isPresent()) {
//                 var target = targetOpt.get();
//                 lastTarget = target;

//                 var cameraPose = robotPose.transformBy(Robot_To_Target);

//                 var camToTarget = target.getBestCameraToTarget();
//                 var targetPose = cameraPose.transformBy(camToTarget);

//                 var goalPose = targetPose.transformBy(Tag_To_Goal).toPose2d();
                
//                 xController.setGoal(goalPose.getX());
//                 yController.setGoal(goalPose.getY());
//                 omegaController.setGoal(photonRes.getBestTarget().getYaw());
//                 SmartDashboard.putNumber("goalPose.getX", goalPose.getX());
//                 SmartDashboard.putNumber("goalPose.getY", goalPose.getY());
//                 SmartDashboard.putNumber("goalPose.getRotation", goalPose.getRotation().getRadians());

//         }
//     }
//     if (lastTarget == null) {
//         drive.stop();
//     } else {
//         var xSpeed = xController.calculate(robotPose.getX());
//         if (xController.atGoal()) {
//             xSpeed = 0;
//         }

//         var ySpeed = yController.calculate(robotPose.getY());
//         if (yController.atGoal()) {
//             ySpeed = 0;
//         }

//         var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
//         if (omegaController.atGoal()) {
//             omegaSpeed = 0;
//         }

//         drive.driveChassisSpeeds(
//             ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose2d.getRotation())
//         );
//     }
//     }
    
//     @Override
//   public boolean isFinished() {
//     return atGoal();
//   }

//   @Override
//   public void end(boolean interrupted) {
//    drive.stop();
//   }

// }
