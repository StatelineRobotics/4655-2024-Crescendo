// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.arm;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double armPos = 0;
  private double armExtenderPos = 0;
    
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
        io.setArmPositions(armPos, armExtenderPos);
    }
  }

  public void requestArmPosition(double armPos, double armExtenderPos) {
      this.armPos = armPos;
      this.armExtenderPos = armExtenderPos;
  } 
  

  private void stop() {
    armPos = 10;
    armExtenderPos = 0;
    io.stop();
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop);
  }

}
