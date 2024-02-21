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
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.mechanisms.MechanismConstants;

public class ArmIOSparkMax implements ArmIO {

    private CANSparkMax m_ArmRight;
    private CANSparkMax  m_ArmLeft;
    private CANSparkMax  m_ArmExtender;
    private AbsoluteEncoder armRightEncoder;
    private RelativeEncoder armExtenderEncoder;
    
    private SparkPIDController armRightController;
    private SparkPIDController armExtenderController;

    public ArmIOSparkMax(){
        m_ArmRight = new CANSparkMax(MechanismConstants.kArmLeftCanId, MotorType.kBrushless);
        m_ArmLeft = new CANSparkMax(MechanismConstants.kArmRightCanId, MotorType.kBrushless);
        m_ArmExtender = new CANSparkMax(MechanismConstants.kArmExtenderCanId, MotorType.kBrushless);
            
        armRightEncoder =  m_ArmRight.getAbsoluteEncoder(Type.kDutyCycle);
        armRightEncoder.setInverted(false);

        armExtenderEncoder = m_ArmExtender.getEncoder();
        
        m_ArmRight.restoreFactoryDefaults();
        m_ArmLeft.restoreFactoryDefaults();
        m_ArmExtender.restoreFactoryDefaults();
        m_ArmRight.setIdleMode(IdleMode.kBrake);
        m_ArmLeft.setIdleMode(IdleMode.kBrake);
        m_ArmExtender.setIdleMode(IdleMode.kBrake);
        m_ArmRight.setInverted(false);
        m_ArmLeft.setInverted(false);
        m_ArmExtender.setInverted(false);
        m_ArmRight.setSmartCurrentLimit(30);
        m_ArmLeft.setSmartCurrentLimit(30);
        m_ArmExtender.setSmartCurrentLimit(30);
    
        armRightController = m_ArmRight.getPIDController();
        armRightController.setFeedbackDevice(armRightEncoder);
        armRightController.setP(1);
        armRightController.setP(0);
        armRightController.setP(0);
        armRightController.setOutputRange(-.25,.25);

        armExtenderController = m_ArmRight.getPIDController();
        armExtenderController.setFeedbackDevice(armExtenderEncoder);
        armExtenderController.setP(1);
        armExtenderController.setP(0);
        armExtenderController.setP(0);
        armExtenderController.setOutputRange(-.25,.25);

        m_ArmRight.burnFlash();
        m_ArmLeft.burnFlash();
        m_ArmExtender.burnFlash();

    }

     @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armRightPos = armRightEncoder.getPosition();
        inputs.armExtenderPos = armExtenderEncoder.getPosition();
        SmartDashboard.putNumber("Arm Postion", armRightEncoder.getPosition()); 
        SmartDashboard.putNumber("Arm Extend Position", armExtenderEncoder.getPosition());
    }

    @Override
    public void setArmMotors(double rightArmPosition) {
       armRightController.setReference(rightArmPosition, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setArmExternerMotor(double armExtenderPostion) {
        armExtenderController.setReference(armExtenderPostion, CANSparkMax.ControlType.kPosition);
    }


}
