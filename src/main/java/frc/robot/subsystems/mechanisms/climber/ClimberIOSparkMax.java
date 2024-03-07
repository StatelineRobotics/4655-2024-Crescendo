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
    private SparkPIDController rightClimberController;
    private SparkPIDController leftClimberController;
    private SparkLimitSwitch rightClimberLimitSwitch;
    private SparkLimitSwitch leftClimberLimitSwitch;
   

    public ClimberIOSparkMax(){
        m_LeftClimber = new CANSparkMax(MechanismConstants.kLeftClimberCanId, MotorType.kBrushless);
        m_RightClimber = new CANSparkMax(MechanismConstants.kRightClimberCanId, MotorType.kBrushless);

        m_LeftClimber.restoreFactoryDefaults();
        m_RightClimber.restoreFactoryDefaults();

        rightClimberEncoder =  m_RightClimber.getEncoder();
        leftClimberEncoder =  m_LeftClimber.getEncoder();

       
        m_LeftClimber.setIdleMode(IdleMode.kCoast);     //MOTORBRAKE
        m_RightClimber.setIdleMode(IdleMode.kCoast);    //MOTORBRAKE
        m_LeftClimber.setInverted(false);
        m_RightClimber.setInverted(false);
        m_LeftClimber.setSmartCurrentLimit(40);
        m_RightClimber.setSmartCurrentLimit(40);
        
        rightClimberLimitSwitch = m_RightClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        leftClimberLimitSwitch = m_LeftClimber.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        
        rightClimberController = m_RightClimber.getPIDController();
        rightClimberController.setFeedbackDevice(rightClimberEncoder);
        rightClimberController.setP(0.00006);
        rightClimberController.setI(0);
        rightClimberController.setD(0);
        rightClimberController.setIZone(0);
        rightClimberController.setFF(0.0003);
        rightClimberController.setOutputRange(-.2,.2);

        leftClimberController = m_RightClimber.getPIDController();
        leftClimberController.setFeedbackDevice(rightClimberEncoder);
        leftClimberController.setP(0.00006);
        leftClimberController.setI(0);
        leftClimberController.setD(0);
        leftClimberController.setIZone(0);
        leftClimberController.setFF(0.0003);
        leftClimberController.setOutputRange(-.2,.2);

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
       leftClimberController.setReference(climberPosition, CANSparkMax.ControlType.kPosition);
       rightClimberController.setReference(climberPosition, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void stop() {
        m_LeftClimber.stopMotor();
        m_RightClimber.stopMotor();
     }
    

}
