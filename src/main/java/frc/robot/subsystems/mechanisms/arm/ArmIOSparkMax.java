// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;


import frc.robot.subsystems.mechanisms.MechanismConstants;

public class ArmIOSparkMax implements ArmIO {

    private CANSparkMax m_ArmRight;
    private CANSparkMax  m_ArmLeft;
    private CANSparkMax  m_ArmExtender;
    private AbsoluteEncoder armEncoder;
    private RelativeEncoder armExtenderEncoder;
    
    private SparkPIDController armController;
    private SparkPIDController armExtenderController;
    private SparkLimitSwitch extendLimitSwitch;

    public ArmIOSparkMax(){
        m_ArmRight = new CANSparkMax(MechanismConstants.kArmLeftCanId, MotorType.kBrushless);
        m_ArmLeft = new CANSparkMax(MechanismConstants.kArmRightCanId, MotorType.kBrushless);
        m_ArmExtender = new CANSparkMax(MechanismConstants.kArmExtenderCanId, MotorType.kBrushless);
            
        m_ArmRight.restoreFactoryDefaults();;
        m_ArmLeft.restoreFactoryDefaults();;
        m_ArmExtender.restoreFactoryDefaults();

        armEncoder =  m_ArmRight.getAbsoluteEncoder(Type.kDutyCycle);
        armEncoder.setInverted(true);
        armEncoder.setPositionConversionFactor(360);

        armExtenderEncoder = m_ArmExtender.getEncoder();
              
   
        m_ArmRight.setIdleMode(IdleMode.kBrake);
        m_ArmLeft.setIdleMode(IdleMode.kBrake);
        m_ArmExtender.setIdleMode(IdleMode.kCoast);
        m_ArmRight.setInverted(false);
        m_ArmLeft.setInverted(false);
        m_ArmExtender.setInverted(true);
        m_ArmRight.setSmartCurrentLimit(40);
        m_ArmLeft.setSmartCurrentLimit(40);
        m_ArmExtender.setSmartCurrentLimit(20);
    
        armController = m_ArmRight.getPIDController();
        armController.setFeedbackDevice(armEncoder);
        armController.setP(.02);
        armController.setI(0);
        armController.setD(0);
        armController.setIZone(0);
        armController.setFF(.000355);
        armController.setOutputRange(-.25,.50);
        

        armExtenderController = m_ArmExtender.getPIDController();
        armExtenderController.setFeedbackDevice(armExtenderEncoder);
        armExtenderController.setP(.00006);
        armExtenderController.setP(0);
        armExtenderController.setP(0);
        armExtenderController.setIZone(0);
        armExtenderController.setFF(0.0025);
        armExtenderController.setOutputRange(-.9,.9);
        int smartMotionSlot = 0;
        armExtenderController.setSmartMotionMaxVelocity(1000, smartMotionSlot);
        armExtenderController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        armExtenderController.setSmartMotionMaxAccel(500, smartMotionSlot);
        armExtenderController.setSmartMotionAllowedClosedLoopError(1, smartMotionSlot);
        extendLimitSwitch = m_ArmExtender.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        extendLimitSwitch.enableLimitSwitch(true);

        m_ArmRight.burnFlash();
        m_ArmLeft.burnFlash();
        m_ArmExtender.burnFlash();

    }

     @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armPos = armEncoder.getPosition();
        inputs.armExtenderPos = armExtenderEncoder.getPosition();
          if(extendLimitSwitch.isPressed()) {
            armExtenderEncoder.setPosition(0);
        }
    }

   

     @Override
    public void setArmPositions(double armPos, double armExtenderPos) {
        armController.setReference(armPos, CANSparkMax.ControlType.kPosition);
        if (armExtenderEncoder.getPosition() > 90 && armPos > 9){
        armExtenderController.setReference(armExtenderPos, CANSparkMax.ControlType.kSmartMotion);
        }
    }


}
