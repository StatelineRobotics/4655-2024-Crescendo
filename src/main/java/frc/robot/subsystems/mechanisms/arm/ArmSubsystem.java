// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.arm;


import frc.robot.subsystems.mechanisms.MechanismConstants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private boolean pickup = false;
  private boolean shootPosition = false;
  private boolean home = false;
  
  public ArmSubsystem(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;
   
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    

    if (DriverStation.isDisabled()) {
      stop();
    } else {
      if (pickup) {
        io.setArmMotors(MechanismConstants.kArmPickupPOS);
        io.setArmExternerMotor(MechanismConstants.kArmExtenderPickupPOS);
      } else if (shootPosition) {
        io.setArmMotors(MechanismConstants.kArmShootPOS);
        io.setArmExternerMotor(MechanismConstants.kArmShootPOS);
      } else if (home) {
        io.setArmMotors(MechanismConstants.kArmHomePOS);
        io.setArmExternerMotor(MechanismConstants.kArmExtenderHomePOS);
      }  
    }

  }

  private void stop() {
    pickup = false;
    shootPosition = false;
    home = false;
    io.stop();
}

}
