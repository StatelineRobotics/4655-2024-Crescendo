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

    // PID coefficients
    public double kP = 0.00049; 
    public double kI = 0;
    public double kD = 0; 
    public double kIz = 0; 
    public double kFF = .00004; 
    public double kMaxOutput = .5; 
    public double kMinOutput = -.5;
    public double maxRPM = 5700;
    public double maxVel = 1500; 
    public double maxAcc = 1000;
    public double minVel = 0;
   

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
        rightClimberController.setP(kP);
        rightClimberController.setI(kI);
        rightClimberController.setD(kD);
        rightClimberController.setIZone(kIz);
        rightClimberController.setFF(kFF);
        rightClimberController.setOutputRange(kMinOutput,  kMaxOutput);
        int smartMotionSlot = 0;
        rightClimberController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        rightClimberController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        rightClimberController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
       

        leftClimberController = m_RightClimber.getPIDController();
        leftClimberController.setFeedbackDevice(rightClimberEncoder);
        leftClimberController.setP(kP);
        leftClimberController.setI(kI);
        leftClimberController.setD(kD);
        leftClimberController.setIZone(kIz);
        leftClimberController.setFF(kFF);
        leftClimberController.setOutputRange(kMinOutput,  kMaxOutput);
        leftClimberController.setSmartMotionMaxVelocity(maxRPM, smartMotionSlot);
        leftClimberController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        leftClimberController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);

        m_LeftClimber.burnFlash();
        m_RightClimber.burnFlash();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.rightClimberPosition = rightClimberEncoder.getPosition();
        inputs.leftClimberPosition = leftClimberEncoder.getPosition();

        

        if(rightClimberLimitSwitch.isPressed()) {
            rightClimberEncoder.setPosition(0);
        }

        if(leftClimberLimitSwitch.isPressed()) {
            leftClimberEncoder.setPosition(0);
        }

    }

    @Override
    public void setclimberMotors(double climberPosition) {
       leftClimberController.setReference(climberPosition, CANSparkMax.ControlType.kSmartMotion);
       rightClimberController.setReference(climberPosition, CANSparkMax.ControlType.kSmartMotion);
    }


    @Override
    public void stop() {
        m_LeftClimber.stopMotor();
        m_RightClimber.stopMotor();
     }
    

}
