// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.arm;

import org.littletonrobotics.junction.AutoLog;


/** Add your docs here. */
public interface ArmIO {
    @AutoLog
    class ArmIOInputs{
        public double armRightPos = 0.0;
        public double armExtenderPos = 0/0; 
    }
  
        /** Update inputs */
    default void updateInputs(ArmIOInputs inputs) {}

        /** Set Arm motor positions */
    default void setArmMotors(double rightArmPosition) {}

    default void setArmExternerMotor(double armExtenderPostion) {}

    /** Stop Arm */
    default void stop() {}   

} 
