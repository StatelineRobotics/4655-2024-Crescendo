// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

 
/** Add your docs here. */
public class ClimberSubsystem  extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged() ;
    private double climberPosition = 0.0;
    private double ClimberSpeed = 0.0;
 


  public ClimberSubsystem(ClimberIO io){
    System.out.println("[Init] Creating Intake");
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    
    if (DriverStation.isDisabled()) {
      stop();
    } else {
      io.setclimberMotors(climberPosition);
      io.setclimberSpeed(ClimberSpeed);
   }
   
   
  }
  
  public void requestClimberPosition(double climberPosition){
    this.climberPosition = climberPosition;
  }



  private void stop() {
    climberPosition = 0;
    io.stop();
  }

   public Command stopCommand() {
    return Commands.runOnce(this::stop);
  }


}
