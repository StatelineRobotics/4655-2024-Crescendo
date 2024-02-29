// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {
   @AutoLog
   class ClimberIOInputs {
    public double climberPosition = 0.0;

   }

  /** Update inputs */
  default void updateInputs(ClimberIOInputs inputs) {}

  /** Set Elevator motor positions */
  default void setclimberMotors(double climberPosition) {}

  /** Stop intake */
  default void stop() {}   
} 
