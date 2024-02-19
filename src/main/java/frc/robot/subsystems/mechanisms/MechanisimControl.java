// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.mechanisms.intake.IntakeSubsystem;

public class MechanisimControl extends SubsystemBase {
 //   private static LoggedTunableNumber wristIntakeSetpointDegrees =
 //     new LoggedTunableNumber("WristIntakeSetpointDegrees", 0.0);
 //   private static LoggedTunableNumber intakeRPM =
 //     new LoggedTunableNumber("IntakeRPM", 0.0);

  public enum State {
    PICKUP,
    SHOOT,
    PREPARE_SHOOT,
    IDLE
  }

  private State currentState = State.IDLE;

  //@Getter private State currentState = State.IDLE;
  private final IntakeSubsystem intakeSubsystem;

  /** Creates a new MechanisimControl. */
  public MechanisimControl(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
  }

  @Override
  public void periodic() {
    switch (currentState) {
      case IDLE -> {
        intakeSubsystem.stopCommand();
       }
      case PICKUP -> {
        intakeSubsystem.pickupCommand();
        }
      case PREPARE_SHOOT -> {
        intakeSubsystem.shootPoitionCommand();
      }
      case SHOOT -> {
        intakeSubsystem.shootCommand();
        }
      
    }

    //Logger.recordOutput("DevBotSuperstructure/currentState", currentState.toString());// This method will be called once per scheduler run
  }

  public void setDesiredState(State desiredState) {
    if (desiredState == State.PREPARE_SHOOT && currentState == State.SHOOT) {
      return;
    }
    currentState = desiredState;
  }
  
}
