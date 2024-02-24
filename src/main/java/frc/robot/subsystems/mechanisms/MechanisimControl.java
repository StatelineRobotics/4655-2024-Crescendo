// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.subsystems.mechanisms.arm.ArmSubsystem;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;
import frc.robot.subsystems.mechanisms.shooter.ShooterSubsystem;
import frc.robot.util.LoggedTunableNumber;

public class MechanisimControl extends SubsystemBase {

  //Home Setting
  private static LoggedTunableNumber wristHomePos =
    new LoggedTunableNumber("Tune WristHomePOS", 10);

  //Shooting Settings    
  private static LoggedTunableNumber wristShootingPos =
    new LoggedTunableNumber("Tune Wrist Whooting POS", 30);
  private static LoggedTunableNumber shootingTopRPM =
    new LoggedTunableNumber("Tune ShootingTopRPM", 6000.0);
  private static LoggedTunableNumber shootingBottomRPM =
    new LoggedTunableNumber("Tune ShootingBottomRPM", 4000.0);
  
  //pickup Settings
  private static LoggedTunableNumber wristPickupPos =
    new LoggedTunableNumber("Tune WristPickupPOS", 20);
  private static LoggedTunableNumber intakePickupRPM =
    new LoggedTunableNumber("Tune IntakePickupRPM", 100);


  public enum State {
    PREPARE_SHOOT,
    SHOOT,
    PICKUP,
    HOME
  }

  private State currentState = State.HOME;

  //@Getter private State currentState = State.IDLE;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  /** Creates a new MechanisimControl. */
  public MechanisimControl(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem ) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
  }

  @Override
  public void periodic() {
    
    switch (currentState) {
      case HOME -> {
        intakeSubsystem.requestIntake(0, wristHomePos.get());
        shooterSubsystem.requestRPM(0, 0);
       }
      case PICKUP -> {
        intakeSubsystem.requestIntake(intakePickupRPM.get(),wristPickupPos.get());
        shooterSubsystem.requestRPM(0, 0);
        }
      case PREPARE_SHOOT -> {
        intakeSubsystem.requestIntake(0,wristShootingPos.get());
        shooterSubsystem.requestRPM(shootingTopRPM.get(), shootingBottomRPM.get());
      }
      case SHOOT -> {
//        if (!atShootingSetpoint()) {
          currentState = State.PREPARE_SHOOT;
//        } else {
           intakeSubsystem.requestIntake(20,wristShootingPos.get());
           shooterSubsystem.requestRPM(shootingTopRPM.get(), shootingBottomRPM.get());
//        }
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
