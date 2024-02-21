// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.mechanisms.MechanismConstants;

/** Add your docs here. */
public class ClimberIOSparkMax implements ClimberIO {
    private CANSparkMax m_LeftClimber;
    private CANSparkMax  m_RightClimber;
    private RelativeEncoder rightClimberEncoder;
    private RelativeEncoder leftClimberEncoder;
    private SparkPIDController climberController;
   

    public ClimberIOSparkMax(){
        m_LeftClimber = new CANSparkMax(MechanismConstants.kLeftClimberCanId, MotorType.kBrushless);
        m_RightClimber = new CANSparkMax(MechanismConstants.kRightClimberCanId, MotorType.kBrushless);

        m_LeftClimber.follow(m_RightClimber);

        rightClimberEncoder =  m_RightClimber.getEncoder();

        m_LeftClimber.restoreFactoryDefaults();
        m_RightClimber.restoreFactoryDefaults();
        m_LeftClimber.setIdleMode(IdleMode.kCoast);
        m_RightClimber.setIdleMode(IdleMode.kCoast);
        m_LeftClimber.setInverted(false);
        m_RightClimber.setInverted(false);
        m_LeftClimber.setSmartCurrentLimit(30);
        m_RightClimber.setSmartCurrentLimit(30);
        
        climberController = m_RightClimber.getPIDController();
        climberController.setFeedbackDevice(rightClimberEncoder);
        climberController.setP(1);
        climberController.setP(0);
        climberController.setP(0);
        climberController.setOutputRange(-.2,.2);

       

        m_LeftClimber.burnFlash();
        m_RightClimber.burnFlash();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.leftClimberPosition = leftClimberEncoder.getPosition();
        inputs.rightClimberPosition = rightClimberEncoder.getPosition();
        SmartDashboard.putNumber("Left Climber Postion", leftClimberEncoder.getPosition()); 
        SmartDashboard.putNumber("Right Climber Position", rightClimberEncoder.getPosition());
    }

    @Override
    public void setElevatorMotors(double rightClimberPosition) {
       climberController.setReference(rightClimberPosition, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setclimbCommand(double percent) {
        m_LeftClimber.set(percent);
    }
    

}
