package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs  {
        public double drivePositionMeters = 0.0;
        public double driveVelocityMetersPerSec = 0.0;

        public Rotation2d turnAbsolutePosition = new Rotation2d(); //radians
        public double turnVelocityRadPerSec = 0.0;
        public Rotation2d drivePosition = new Rotation2d();
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double[] driveCurrentAmps = new double[] {};
    
        public Rotation2d turnPosition = new Rotation2d();
        public double turnAppliedVolts = 0.0;
        public double[] turnCurrentAmps = new double[] {};
        public Rotation2d angularOffset = new Rotation2d();
    }

    public default void updateInputs(ModuleIOInputs inputs){}

    public default void setDesiredState(SwerveModuleState state){}

    public default void resetEncoders(){}

    public default SwerveModuleState getOptimizedState(){
        return null;
    }
    public default void periodic() {}
}
