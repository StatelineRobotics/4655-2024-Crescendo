// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.intake;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    public double intakeVelocityRPM = 0.0;
    public double intakeAppliedVolts = 0.0;
    public double intakeOutputCurrent = 0.0;

    public double wristPosition = 0.0;
    public double wristAppliedVolts = 0.0;
    public double wristOutputCurrent = 0.0;

  }

  /** Update inputs */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Set voltage of intake */
  default void setIntakeRPM(double rpm) {}

  /** Set position of wrist */
  default void setwristPostion (double angle) {}

  /** Set brake mode of intake */
  default void setIntakeBrakeMode(boolean enabled) {}


  /** Stop intake */
  default void stop() {}
}
