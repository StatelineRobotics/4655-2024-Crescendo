// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.ShooterAlignments;
import frc.robot.subsystems.mechanisms.arm.ArmSubsystem;
import frc.robot.subsystems.mechanisms.climber.ClimberSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.ShooterAlignments;

public class MechanisimControl extends SubsystemBase {
 
  public enum State {
    PREPARE_SHOOT,
    SHOOT,
    PICKUP,
    HOME,
    CLIMB,
    CLIMBSHOOT,
    GRAB,
    AMP,
    AMPSHOOT,
    EJECT,
    AUTO_AIM
  }

  private State currentState = State.HOME;

  //@Getter private State currentState = State.IDLE;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final ShooterAlignments shooterAlignments;

  /** Creates a new MechanisimControl. */
  public MechanisimControl(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem, ClimberSubsystem climberSubsystem, ShooterAlignments shooterAlignments) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.armSubsystem = armSubsystem;
    this.climberSubsystem = climberSubsystem; 
    this.shooterAlignments = shooterAlignments;
  }

  @Override
  public void periodic() {
    
    switch (currentState) {
      case HOME -> {
          if (armSubsystem.OkToHome()){ 
            intakeSubsystem.requestIntake(0, 50);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(24, 0);
            climberSubsystem.requestClimberPosition(0);
            intakeSubsystem.requestBlinken(0.53);
          } else{
            intakeSubsystem.requestIntake(0, 35);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(25, 115);
            climberSubsystem.requestClimberPosition(0);
            intakeSubsystem.requestBlinken(0.53);
          }
       }

      case EJECT -> {
          if (armSubsystem.OkToHome()){ 
            if (intakeSubsystem.OkToEject()) {
              intakeSubsystem.requestIntake(-600, 50);
            } else {
              intakeSubsystem.requestIntake(0, 50);
            }
                        shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(24, 0);
            climberSubsystem.requestClimberPosition(0);
            intakeSubsystem.requestBlinken(0.53);
            } else {
            intakeSubsystem.requestIntake(0, 35);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(25, 115);
            climberSubsystem.requestClimberPosition(0);
            intakeSubsystem.requestBlinken(0.53);
            }
        
       } 
  
      case AMP -> {
        intakeSubsystem.requestIntake(0, 158);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(64, 40);
        climberSubsystem.requestClimberPosition(0);
        intakeSubsystem.requestBlinken(0.53);

       }

      case AMPSHOOT -> {
        intakeSubsystem.requestIntake(600, 158);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(64, 40);
        climberSubsystem.requestClimberPosition(0);
        intakeSubsystem.requestBlinken(0.53);

       } 

      case PICKUP -> {
          if (armSubsystem.OkToPickup()){ 
        
            intakeSubsystem.requestIntake(450, 25);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(10, 127);
              if(intakeSubsystem.HasNote()){
                intakeSubsystem.requestBlinken(-.05);
              } else {
                intakeSubsystem.requestBlinken(0.53);
              }

            //climberSubsystem.requestClimberPosition(0);
          } else {
            intakeSubsystem.requestIntake(0, 35);
            shooterSubsystem.requestRPM(0, 0);
            armSubsystem.requestArmPosition(25, 115);
            //climberSubsystem.requestClimberPosition(0);
            intakeSubsystem.requestBlinken(0.53); 
          }

        }

      case PREPARE_SHOOT -> {
        intakeSubsystem.requestIntake(0,238);
        shooterSubsystem.requestRPM(5600, 5900);
        armSubsystem.requestArmPosition(9.5, 10);
        climberSubsystem.requestClimberPosition(0);
        intakeSubsystem.requestBlinken(0.53);
      }

        case AUTO_AIM -> {
          double armAngle = shooterAlignments.angleArmToSpeaker();
          intakeSubsystem.requestIntake(0,238);
          shooterSubsystem.requestRPM(5600, 5900);
          armSubsystem.requestArmPosition(armAngle, 10);
          climberSubsystem.requestClimberPosition(0);
          intakeSubsystem.requestBlinken(0.53);
      }


      case SHOOT -> {
//        if (!atShootingSetpoint()) {
//          currentState = State.PREPARE_SHOOT;
//        } else {
          //  double armAngle = shooterAlignments.angleArmToSpeaker();
          //  intakeSubsystem.requestIntake(-500,238);
          //  shooterSubsystem.requestRPM(5600, 5900);
          //  armSubsystem.requestArmPosition(armAngle, 10);
          //  intakeSubsystem.requestBlinken(0.53);
//        }
      }
      case CLIMB -> {
        if (armSubsystem.OkToClimbWrist()){
          intakeSubsystem.requestIntake(0,165);
        } else {
          intakeSubsystem.requestIntake(0,50);
        }
        
        shooterSubsystem.requestRPM(0, 0);
          if (climberSubsystem.ClimberOkToReach()) {
            armSubsystem.requestArmPosition(86.5, 43);
          }
          else {
            armSubsystem.requestArmPosition(110, 0);
          }
        climberSubsystem.requestClimberPosition(0);
        intakeSubsystem.requestBlinken(-0.57);
      }

      case CLIMBSHOOT -> {
        intakeSubsystem.requestIntake(500,165);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(86.5, 43);
        climberSubsystem.requestClimberPosition(0);
        intakeSubsystem.requestBlinken(-.57);
      }

      case GRAB -> {
        intakeSubsystem.requestIntake(0,50 );
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(110,0);
        climberSubsystem.requestClimberPosition(180);
        intakeSubsystem.requestBlinken(-.57);
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
