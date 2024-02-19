// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.intake;

import org.littletonrobotics.junction.Logger;
import frc.robot.subsystems.mechanisms.MechanismConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io; 
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private boolean pickup = false;
  private boolean shoot = false;
  private boolean shootPosition = false;
  


  public IntakeSubsystem(IntakeIO io) {
    System.out.println("[Init] Creating Intake");
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

 
    if (DriverStation.isDisabled()) {
      stop();
    } else {
      if (pickup) {
        io.setIntakeMotors(MechanismConstants.kIntakePickupRPM, MechanismConstants.kIntakePickupPostion);
      } else if (shootPosition) {
        io.setIntakeMotors(0, MechanismConstants.kIntakePickupPostion);
      } else if (shoot) {
        io.setIntakeMotors(MechanismConstants.kIntakeShootRPM, MechanismConstants.kIntakePickupPostion);
      }  
    }

  }
  
  public boolean pickuping() {
    return pickup;
  }
    
  public boolean shootingPositioning() {
    return shootPosition;
  }

  public boolean shooting() {
    return shoot;
  }


  public boolean running() {
    return pickup || shootPosition || shoot;
  }

  private void pickup() {
    pickup = true;
    shootPosition = false;
    shoot = false;
  }

  private void shootPosition() {
    pickup = false;
    shootPosition = true;
    shoot = false;
  }

  private void shoot() {
    pickup = false;
    shootPosition = false;
    shoot = true;
  }

  private void stop() {
    pickup = false;
    shootPosition = false;
    shoot = false;
    io.stop();
  }

  public Command pickupCommand() {
    return Commands.runOnce(this::pickup);
  }

  public Command shootPoitionCommand() {
    return Commands.runOnce(this::shootPosition);
  }

  public Command shootCommand() {
    return Commands.runOnce(this::shoot);
  }

   public Command stopCommand() {
    return Commands.runOnce(this::stop);
  }

}
