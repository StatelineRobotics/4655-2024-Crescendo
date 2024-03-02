// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.arm;

import org.littletonrobotics.junction.AutoLog;


/** Add your docs here. */
public interface ArmIO {
    @AutoLog
    class ArmIOInputs{
        public double armPos = 0.0;
        public double armExtenderPos = 0.0; 
        public boolean armExtendOK = false;
    }
  
    /** Update inputs */
    default void updateInputs(ArmIOInputs inputs) {}
    
    default void setArmPositions(double armPos, double armExtenderPos) {}

    /** Stop Arm */
    default void stop() {}   

} 
