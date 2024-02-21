// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.shooter;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double TopShooterRPM = 0.0;
    public double BottomShooterRPM = 0.0;
 
  }

  /** Update inputs */
  default void updateInputs(ShooterIOInputs inputs) {}

  /** Set Input speed and position */
  default void setShooterMotors(double shooterTopRPM, double shooterBottomRPM) {}

  /** Stop Shooter */
  default void stop() {}
}