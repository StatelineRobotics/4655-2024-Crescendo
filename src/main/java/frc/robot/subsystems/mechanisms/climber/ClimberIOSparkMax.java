// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;


import frc.robot.subsystems.mechanisms.MechanismConstants;

/** Add your docs here. */
public class ClimberIOSparkMax implements ClimberIO {
    private CANSparkMax m_LeftClimber;
    private CANSparkMax  m_RightClimber;
    private RelativeEncoder rightClimberEncoder;
    private RelativeEncoder leftClimberEncoder;
    private SparkPIDController climberController;
    private SparkLimitSwitch rightClimberLimitSwitch;
    private SparkLimitSwitch leftClimberLimitSwitch;
   

    public ClimberIOSparkMax(){
        m_LeftClimber = new CANSparkMax(MechanismConstants.kLeftClimberCanId, MotorType.kBrushless);
        m_RightClimber = new CANSparkMax(MechanismConstants.kRightClimberCanId, MotorType.kBrushless);

        m_LeftClimber.follow(m_RightClimber);

        rightClimberEncoder =  m_RightClimber.getEncoder();

       
        m_LeftClimber.setIdleMode(IdleMode.kCoast);
        m_RightClimber.setIdleMode(IdleMode.kCoast);
        m_LeftClimber.setInverted(false);
        m_RightClimber.setInverted(false);
        m_LeftClimber.setSmartCurrentLimit(40);
        m_RightClimber.setSmartCurrentLimit(40);
        
        rightClimberLimitSwitch = m_RightClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        leftClimberLimitSwitch = m_LeftClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        
        climberController = m_RightClimber.getPIDController();
        climberController.setFeedbackDevice(rightClimberEncoder);
        climberController.setP(1);
        climberController.setI(0);
        climberController.setD(0);
        climberController.setIZone(0);
        climberController.setFF(0);
        climberController.setOutputRange(-.2,.2);

        m_LeftClimber.burnFlash();
        m_RightClimber.burnFlash();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberPosition = rightClimberEncoder.getPosition();

        

        if(rightClimberLimitSwitch.isPressed()) {
            rightClimberEncoder.setPosition(0);
        }

        if(leftClimberLimitSwitch.isPressed()) {
            leftClimberEncoder.setPosition(0);
        }

    }

    @Override
    public void setclimberMotors(double climberPosition) {
       climberController.setReference(climberPosition, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void stop() {
        m_LeftClimber.stopMotor();
        m_RightClimber.stopMotor();
     }
    

}
