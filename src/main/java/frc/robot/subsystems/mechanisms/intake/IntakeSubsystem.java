// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import frc.robot.subsystems.mechanisms.MechanismConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
    private CANSparkMax m_Wrist;
    private CANSparkMax  m_Intake;
    private AbsoluteEncoder wristEncoder;
    private RelativeEncoder intakeEncoder;
    private SparkPIDController wristController;
    private SparkPIDController intakeController;

  /** Creates a new IntakeSubsustem. */
  public IntakeSubsystem() {
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
    intakeController.setOutputRange(-.05,.05);

    wristController = m_Wrist.getPIDController();
    wristController.setFeedbackDevice(wristEncoder);
    wristController.setP(1);
    wristController.setP(0);
    wristController.setP(0);
    wristController.setOutputRange(-.05,.05);

    m_Wrist.burnFlash();
    m_Intake.burnFlash();

  }

  public double getWristPosition() {
    return wristEncoder.getPosition();
  }

  public double getIntakeRPM() {
    return intakeEncoder.getVelocity();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Wrist Position", getWristPosition());
    SmartDashboard.putNumber("Intake RPM", getIntakeRPM());
    double dsSetIntakeRPM =
          SmartDashboard.getNumber("Set Intake RPM", 0.0);
    double dsSetWristPostion =
          SmartDashboard.getNumber("Set Wrist Positon", 0.0);
   
    if (SmartDashboard.getBoolean("Run Intake", false)) {
      intakeController.setReference(dsSetIntakeRPM,CANSparkMax.ControlType.kVelocity);
    }
    else {
       m_Intake.set(0);
    }    

     if (SmartDashboard.getBoolean("Run Intake", false)) {
      wristController.setReference(dsSetWristPostion, ControlType.kDutyCycle);
    }
    else {
       m_Wrist.set(0);
    }   
    
  }
}
