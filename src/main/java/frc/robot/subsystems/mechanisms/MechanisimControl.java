// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.arm.ArmSubsystem;
import frc.robot.subsystems.mechanisms.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;

public class MechanisimControl extends SubsystemBase {
 
  public enum State {
    PREPARE_SHOOT,
    SHOOT,
    PICKUP,
    HOME,
    CLIMB,
    GRAB,
    MOVE
  }

  private State currentState = State.HOME;

  //@Getter private State currentState = State.IDLE;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ClimberSubsystem climberSubsystem;

  /** Creates a new MechanisimControl. */
  public MechanisimControl(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem, ClimberSubsystem climberSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.armSubsystem = armSubsystem;
    this.climberSubsystem = climberSubsystem; 
  }

  @Override
  public void periodic() {
    
    switch (currentState) {
      case HOME -> {
        intakeSubsystem.requestIntake(0, 25);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(22, 0);
        climberSubsystem.requestClimberPosition(0);

       }

  
        case MOVE -> {
        intakeSubsystem.requestIntake(0, 10);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(25, 100);
        //climberSubsystem.requestClimberPosition(0);

       }

      case PICKUP -> {
          if (intakeSubsystem.OkToPickup()){ 
        
            intakeSubsystem.requestIntake(350, 3);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(10, 115);
            //climberSubsystem.requestClimberPosition(0);
          } else {
            intakeSubsystem.requestIntake(0, 10);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(25, 100);
            //climberSubsystem.requestClimberPosition(0); 
          }

        }

      case PREPARE_SHOOT -> {
        intakeSubsystem.requestIntake(0,212);
        shooterSubsystem.requestRPM(5600, 5900);
        armSubsystem.requestArmPosition(9.5, 10);
        //climberSubsystem.requestClimberPosition(0);
      }


      case SHOOT -> {
//        if (!atShootingSetpoint()) {
//          currentState = State.PREPARE_SHOOT;
//        } else {
           intakeSubsystem.requestIntake(-500,212);
           shooterSubsystem.requestRPM(5600, 5900);
           armSubsystem.requestArmPosition(9.5, 10);
//        }
      }
      case CLIMB -> {
        intakeSubsystem.requestIntake(0,212);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(140, 10);
        climberSubsystem.requestClimberPosition(0);
      }
      case GRAB -> {
        intakeSubsystem.requestIntake(0,212);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(140, 10);
        climberSubsystem.requestClimberPosition(120);
      }
    }

    Logger.recordOutput("MechanisimControl/currentState", currentState.toString());
    
  }
  
  @AutoLogOutput(key = "MechanisimControl/ReadyToShoot")
  public boolean atShootingSetpoint() {
    return (currentState == State.PREPARE_SHOOT || currentState == State.SHOOT)
        //&& ArmSubsystem.atSetpoint()
        && shooterSubsystem.atSetpoint();
  }
  

  public void setDesiredState(State desiredState) {
//    if (desiredState == State.PREPARE_SHOOT && currentState == State.SHOOT) {
//      return;
//    }
    currentState = desiredState;
    
  }

  

  
}
