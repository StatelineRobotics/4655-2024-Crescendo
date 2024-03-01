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
import frc.robot.util.LoggedTunableNumber;

public class MechanisimControl extends SubsystemBase {

  
 

  //climb Settings
  private static LoggedTunableNumber wristClimbPos =
    new LoggedTunableNumber("Tune Wrist Climb POS", 30);
    private static LoggedTunableNumber armClimbPos =
    new LoggedTunableNumber("Tune Arm Climb POS", 20);
  private static LoggedTunableNumber armExtenderClimbPos =
    new LoggedTunableNumber("Tune ArmExterder Climb POS", 0);
  private static LoggedTunableNumber climberGrabPos =
    new LoggedTunableNumber("Tune Climber Grab POS", 0);
  private static LoggedTunableNumber climberElevatePos =
    new LoggedTunableNumber("Tune Climber E;evate POS", 0);


  public enum State {
    PREPARE_SHOOT,
    PREPARE_PICKUP,
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
        intakeSubsystem.requestIntake(0, 75);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(20, 0);
        climberSubsystem.requestClimberPosition(0);

       }

      case PREPARE_PICKUP -> {
        intakeSubsystem.requestIntake(0, 75);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(25, 100);
        climberSubsystem.requestClimberPosition(0);

       }

        case MOVE -> {
        intakeSubsystem.requestIntake(0, 75);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(25, 0);
        climberSubsystem.requestClimberPosition(0);

       }

      case PICKUP -> {
        intakeSubsystem.requestIntake(100, 90);
        shooterSubsystem.requestRPM(0, 0);
        armSubsystem.requestArmPosition(5, 100);
        climberSubsystem.requestClimberPosition(0);
        }

      case PREPARE_SHOOT -> {
        intakeSubsystem.requestIntake(0,wristShootingPos.get());
        shooterSubsystem.requestRPM(shootingTopRPM.get(), shootingBottomRPM.get());
        armSubsystem.requestArmPosition(armShootingPos.get(), armExtenderShootingPos.get());
        climberSubsystem.requestClimberPosition(0);
      }


       //Shooting Settings    
  private static LoggedTunableNumber wristShootingPos =
    new LoggedTunableNumber("Tune Wrist Whooting POS", 240);
  private static LoggedTunableNumber shootingTopRPM =
    new LoggedTunableNumber("Tune Shooting Top RPM", 5600);
  private static LoggedTunableNumber shootingBottomRPM =
    new LoggedTunableNumber("Tune Shooting Bottom RPM", 5900);
  private static LoggedTunableNumber armShootingPos =
    new LoggedTunableNumber("Tune Arm Shooting POS", 10);
  private static LoggedTunableNumber armExtenderShootingPos =
    new LoggedTunableNumber("Tune ArmExterder Shooting POS", 0);
  
  
 

      case SHOOT -> {
//        if (!atShootingSetpoint()) {
          currentState = State.PREPARE_SHOOT;
//        } else {
           intakeSubsystem.requestIntake(20,240;
           shooterSubsystem.requestRPM(5600, 5900);
           armSubsystem.requestArmPosition(armShootingPos.get(), armExtenderShootingPos.get());
//        }
      }
      case CLIMB -> {
        intakeSubsystem.requestIntake(0,wristClimbPos.get());
        shooterSubsystem.requestRPM(shootingTopRPM.get(), shootingBottomRPM.get());
        armSubsystem.requestArmPosition(armClimbPos.get(), armExtenderClimbPos.get());
        climberSubsystem.requestClimberPosition(climberElevatePos.get());
      }
      case GRAB -> {
        intakeSubsystem.requestIntake(0,240);
        shooterSubsystem.requestRPM(shootingTopRPM.get(), shootingBottomRPM.get());
        armSubsystem.requestArmPosition(armClimbPos.get(), armExtenderClimbPos.get());
        climberSubsystem.requestClimberPosition(climberGrabPos.get());
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
    if (desiredState == State.PREPARE_SHOOT && currentState == State.SHOOT) {
      return;
    }
    currentState = desiredState;
  }
  
}
