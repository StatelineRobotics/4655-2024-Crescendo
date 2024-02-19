// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.climber;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.mechanisms.MechanismConstants;


/** Add your docs here. */
public class ClimberSubsystem  extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged() ;
    private boolean chainGrab = false;
    private boolean climb = false;
    private boolean reset = false;


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
            if (chainGrab) {
                io.setElevatorMotors(MechanismConstants.kClimberGrabPosition);
            } else if (climb) {
                io.setElevatorMotors(MechanismConstants.kClimberClimbPosition);
            } else if (reset) {
                io.setElevatorMotors(MechanismConstants.kClimberResetPosition);
            }  
        }
    }

    public boolean grabing() {
        return chainGrab;
    }
        
    public boolean climbing() {
        return climb;
    }
    
    public boolean reseting() {
        return reset;
    }

    public boolean running() {
        return chainGrab || climb || reset;
      }
    
    private void chainGrab() {
        chainGrab = true;
        climb = false;
        reset = false;
    }
    
    private void climb() {
        chainGrab = false;
        climb = true;
        reset = false;
    }
    
    private void reset() {
        chainGrab = false;
        climb = false;
        reset = true;
    }
    
    private void stop() {
        chainGrab = false;
        climb = false;
        reset = false;
        io.stop();
    }

   public Command chainGrabCommand() {
    return Commands.runOnce(this::chainGrab);
  }

  public Command climbCommand() {
    return Commands.runOnce(this::climb);
  }

  public Command resetCommand() {
    return Commands.runOnce(this::reset);
  }

   public Command stopCommand() {
    return Commands.runOnce(this::stop);
  }
 
 
}
