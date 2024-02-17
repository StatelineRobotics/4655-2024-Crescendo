// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.subsystems.mechanisms.MechanismConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsustem extends SubsystemBase {
  
    private CANSparkMax m_Wrist;
    private CANSparkMax  m_Intake;
    private AbsoluteEncoder wristEncoder;
    private RelativeEncoder intakeEncoder;
    private SparkPIDController wristController;
    private SparkPIDController intakeController;

  /** Creates a new IntakeSubsustem. */
  public IntakeSubsustem() {
    m_Wrist = new CANSparkMax(MechanismConstants.kIntakeCanId, MotorType.kBrushless);
    m_Intake = new CANSparkMax(MechanismConstants.kWristCanId, MotorType.kBrushless);
    
    wristEncoder =  m_Wrist.getAbsoluteEncoder(Type.kDutyCycle);
    intakeEncoder =  m_Intake.getEncoder();
    wristEncoder.setInverted(false);

    m_Wrist.restoreFactoryDefaults();
    m_Intake.restoreFactoryDefaults();
    m_Wrist.setIdleMode(IdleMode.kBrake);
    m_Intake.setIdleMode(IdleMode.kCoast);
    m_Wrist.setInverted(false);
    m_Intake.setInverted(false);
    m_Wrist.setSmartCurrentLimit(30);
    m_Intake.setSmartCurrentLimit(30);
 
    intakeController = m_Intake.getPIDController();
    intakeController.setFeedbackDevice(intakeEncoder);
    intakeController.setP(1);
    intakeController.setP(0);
    intakeController.setP(0);
    intakeController.setOutputRange(-.25,.25);

    wristController = m_Wrist.getPIDController();
    wristController.setFeedbackDevice(wristEncoder);
    wristController.setP(1);
    wristController.setP(0);
    wristController.setP(0);
    wristController.setOutputRange(-.25,.25);

    m_Wrist.burnFlash();
    m_Intake.burnFlash();

  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
