// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.arm;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.MechanismConstants;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double armPosSet = MechanismConstants.kArmHomePOS;
  private double armExtenderPosSet = MechanismConstants.kArmExtenderHomePOS;;
    
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
        io.setArmPositions(armPosSet, armExtenderPosSet);
    }
    
  }

  public void requestArmPosition(double armPosSet, double armExtenderPosSet) {
    this.armPosSet = armPosSet;
    this.armExtenderPosSet = armExtenderPosSet;

  }

   

  private void stop() {
    //armPos = 15;
    //armExtenderPos = 0;
    io.stop();
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop);
  }




}
