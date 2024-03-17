// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive.DriveConstants.ModuleConstants;
import frc.robot.subsystems.Vision.PhotonVision;
import frc.robot.subsystems.Vision.ShooterAlignments;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive extends SubsystemBase {

    private final Module m_frontLeft;
    private final Module m_frontRight;
    private final Module m_rearLeft;
    private final Module m_rearRight;


    // The gyro sensor
    private GyroIO gyro;
    private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    // Slew rate filter variables for controlling lateral acceleration
    private double m_currentRotation = 0.0;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;

    private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;

    // Odometry class for tracking robot pose
    SwerveDrivePoseEstimator poseEstimator;
    SwerveDriveOdometry m_odometry;   
    private Pose2d pose = new Pose2d();
    private Pose2d Opose = new Pose2d();
    private Pose2d Vpose = new Pose2d();
    private PhotonVision photonVision;
    private Field2d field = new Field2d();
    private boolean update = false;
    private double VisionTime;
   

    /** Creates a new DriveSubsystem. */
    public Drive(GyroIO gyro, ModuleIO fl, ModuleIO fr, ModuleIO bl, ModuleIO br,PhotonVision photonVision) {
        this.gyro = gyro;
        this.photonVision = photonVision;
        m_frontLeft = new Module(fl, 0);
        m_frontRight = new Module(fr, 1);
        m_rearLeft = new Module(bl, 2);
        m_rearRight = new Module(br, 3);

        m_frontLeft.updateInputs();
        m_frontRight.updateInputs();
        m_rearLeft.updateInputs();
        m_rearRight.updateInputs();


        m_odometry = new SwerveDriveOdometry(      
                DriveConstants.kDriveKinematics,
                gyroInputs.yaw.plus(DriveConstants.kChassisAngularOffset),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                });  

                
        poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            gyroInputs.yaw.plus(DriveConstants.kChassisAngularOffset),
            new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
            },new Pose2d());

        this.zeroHeading();

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(ModuleConstants.kAutoDrivingP, ModuleConstants.kAutoDrivingI,
                                ModuleConstants.kAutoDrivingD), // Translation PID constants
                        new PIDConstants(ModuleConstants.kAutoTurningP, ModuleConstants.kAutoTurningI,
                                ModuleConstants.kAutoTurningD), // Rotation PID constants
                        4.5, // Max module speed, in m/s ***MIGHT CHANGE***
                        DriveConstants.kTrackRadius, // Drive base radius in meters. Distance from robot center to
                                                     // furthest module. ***MIGHT CHANGE***
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                
                () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red,
                this // Reference to this subsystem to set requirements
        );

        Pathfinding.setPathfinder(new LocalADStarAK()); // Implements A* pathfinding algorithm (very cool btw) -
                                                        // assuming I understand this correctly

        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                }); // Adds a way for PathPlanner to log what poses it's trying to get the robot to
                    // go to

        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                }); // Adds a way for PathPlanner to log what pose it's currently trying to go to
    }

    @Override
    public void periodic() {

        Optional<Pose2d> estimatedPose = photonVision.getEstimatedPose(getPose());
        if (estimatedPose.isPresent() && photonVision.getPoseAmbiguity()){
        var odometrypose = poseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("OdometryPoseX", odometrypose.getX());
        SmartDashboard.putNumber("OdometryPoseY", odometrypose.getY());
        Vpose = estimatedPose.get();
        field.setRobotPose(getPose());
        SmartDashboard.putString("Pose", getPose().toString());
        Logger.recordOutput("Odometry", getPose());
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        SmartDashboard.putNumber("OdometryPoseX2", Opose.getX());
        SmartDashboard.putNumber("OdometryPoseY2", Opose.getY());
     } 
     if (photonVision.getLatestResult().hasTargets()){
        Opose = new Pose2d(new Translation2d(Vpose.getX(),Vpose.getY()),gyroInputs.yaw);
        VisionTime = photonVision.getLatestResult().getTimestampSeconds();
        update = true;
     }
     if (update){
        poseEstimator.addVisionMeasurement(estimatedPose.get(), photonVision.getTimestamp());
        SetVisionPose();
        if (VisionTime < photonVision.getLatestResult().getTimestampSeconds()){
            update = false;
        }
    }

        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);
        m_frontLeft.updateInputs();
        m_frontRight.updateInputs();
        m_rearLeft.updateInputs();
        m_rearRight.updateInputs();
               
       // Updates both odometry in the periodic block 
        m_odometry.update(
            gyroInputs.yaw.plus(DriveConstants.kChassisAngularOffset),
            new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
            });

        poseEstimator.update(
            gyroInputs.yaw.plus(DriveConstants.kChassisAngularOffset),
            new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
            });
  
            SmartDashboard.putString("killurparents",m_rearLeft.getPosition().toString());
            SmartDashboard.putString("killurparentspls",m_frontLeft.getPosition().toString());


        // Read wheel deltas from each module
        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        wheelDeltas[0] = m_frontLeft.getPositionDelta();
        wheelDeltas[1] = m_frontRight.getPositionDelta();
        wheelDeltas[2] = m_rearLeft.getPositionDelta();
        wheelDeltas[3] = m_rearRight.getPositionDelta();

        // The twist represents the motion of the robot since the last
        // sample in x, y, and theta based only on the modules, without
        // the gyro.
        var twist = DriveConstants.kDriveKinematics.toTwist2d(wheelDeltas);
        // Apply the twist (change since last sample) to the current pose
        pose = pose.exp(twist);

       

        Logger.recordOutput("Odometry", getPose());
        Logger.recordOutput("Simulated Pose", pose);
        Logger.recordOutput("Swerve/SwerveStates", this.getModuleStates());
        Logger.recordOutput("Swerve/OptimizedStates", this.getOptimizedStates());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition(); 
    }

    public Pose2d getVPose() {
        return Opose; 
    }


    public void SetVisionPose(){
        poseEstimator.resetPosition(
            gyroInputs.yaw.plus(DriveConstants.kChassisAngularOffset),
            new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
            },getVPose());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(
                gyroInputs.yaw.plus(DriveConstants.kChassisAngularOffset),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_rearLeft.getPosition(),
                        m_rearRight.getPosition()
                },
                pose);

        this.pose = pose;
    }



    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     * @param rateLimit     Whether to enable rate limiting for smoother control.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

        double xSpeedCommanded;
        double ySpeedCommanded;

        if (rateLimit) {
            // Convert XY to polar for rate limiting
            double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
            double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

            // Calculate the direction slew rate based on an estimate of the lateral
            // acceleration
            double directionSlewRate;
            if (m_currentTranslationMag != 0.0) {
                directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
            } else {
                directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
            }

            double currentTime = WPIUtilJNI.now() * 1e-6;
            double elapsedTime = currentTime - m_prevTime;
            double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
            if (angleDif < 0.45 * Math.PI) {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
            } else if (angleDif > 0.85 * Math.PI) {
                if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality
                                                      // checking
                    // keep currentTranslationDir unchanged
                    m_currentTranslationMag = m_magLimiter.calculate(0.0);
                } else {
                    m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
                    m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
                }
            } else {
                m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
                        directionSlewRate * elapsedTime);
                m_currentTranslationMag = m_magLimiter.calculate(0.0);
            }
            m_prevTime = currentTime;

            
            xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
            ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
            m_currentRotation = m_rotLimiter.calculate(rot);

        } else {
            xSpeedCommanded = xSpeed;
            ySpeedCommanded = ySpeed;
            m_currentRotation = rot;

        }

        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

        Logger.recordOutput("Swerve/FieldDelative", fieldRelative);
        Logger.recordOutput("Swerve/XspeedCommanded", xSpeedDelivered);
        Logger.recordOutput("Swerve/YspeedCommanded", ySpeedDelivered);

         SmartDashboard.putBoolean("Field Relative", fieldRelative);

         
        Rotation2d fieldRelativeRotation;
        switch(Constants.currentMode){
            case REAL:
                fieldRelativeRotation = gyroInputs.yaw;
                break;
            case SIM:
                fieldRelativeRotation = pose.getRotation();
                break;
            default:
                fieldRelativeRotation = new Rotation2d();
                break;

        }

        boolean isFlipped =
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                        isFlipped
                            ?   fieldRelativeRotation.plus(new Rotation2d(Math.PI))
                            :   fieldRelativeRotation)
                            // was fieldRelativeRotation.plus(DriveConstants.kChassisAngularOffset)
                        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45 )));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
       
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] currentState = new SwerveModuleState[4];
        currentState[0] = m_frontLeft.getState();
        currentState[1] = m_frontRight.getState();
        currentState[2] = m_rearLeft.getState();
        currentState[3] = m_rearRight.getState();
        return currentState;
    }

    public SwerveModuleState[] getOptimizedStates() {
        SwerveModuleState[] currentState = new SwerveModuleState[4];
        currentState[0] = m_frontLeft.getOptimizedState();
        currentState[1] = m_frontRight.getOptimizedState();
        currentState[2] = m_rearLeft.getOptimizedState();
        currentState[3] = m_rearRight.getOptimizedState();
        return currentState;
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
        pose.rotateBy(pose.getRotation().times(-1)); //This may not work
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyroInputs.yaw.getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return gyroInputs.yawVelocity;
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        // Uses forward kinematics to calculate the robot's speed given the states of
        // the swerve modules.
        return DriveConstants.kDriveKinematics.toChassisSpeeds(m_frontLeft.getState(), m_frontRight.getState(),
                m_rearLeft.getState(), m_rearRight.getState());
    }

    private void driveRobotRelative(ChassisSpeeds speeds) {
        // This takes the velocities and converts them into precentages (-1 to 1)
        drive(speeds.vxMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
                speeds.vyMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond,
                speeds.omegaRadiansPerSecond / DriveConstants.kMaxAngularSpeed,
                false,
                false);
    }
}
