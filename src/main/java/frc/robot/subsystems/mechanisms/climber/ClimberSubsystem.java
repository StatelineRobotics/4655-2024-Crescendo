// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.subsystems.mechanisms.MechanismConstants;


/** Add your docs here. */
public class ClimberSubsystem  extends SubsystemBase {
    private CANSparkMax m_LeftClimber;
    private CANSparkMax  m_RightClimber;
    private RelativeEncoder leftClimberEncoder;
    private RelativeEncoder rightClimberEncoder;
    private SparkPIDController rightClimberController;
    private SparkPIDController leftClimberController;

    public ClimberSubsystem(){

         m_LeftClimber = new CANSparkMax(MechanismConstants.kLeftClimberCanId, MotorType.kBrushless);
         m_RightClimber = new CANSparkMax(MechanismConstants.kRightClimberCanId, MotorType.kBrushless);

        leftClimberEncoder =  m_LeftClimber.getEncoder();
        rightClimberEncoder =  m_RightClimber.getEncoder();

         m_LeftClimber.restoreFactoryDefaults();
         m_RightClimber.restoreFactoryDefaults();
         m_LeftClimber.setIdleMode(IdleMode.kBrake);
         m_RightClimber.setIdleMode(IdleMode.kBrake);
         m_LeftClimber.setInverted(false);
         m_RightClimber.setInverted(true);
         m_LeftClimber.setSmartCurrentLimit(30);
         m_RightClimber.setSmartCurrentLimit(30);
        
        leftClimberController = m_LeftClimber.getPIDController();
        leftClimberController.setFeedbackDevice(leftClimberEncoder);
        leftClimberController.setP(1);
        leftClimberController.setP(0);
        leftClimberController.setP(0);
        leftClimberController.setOutputRange(-.25,.25);

        rightClimberController = m_RightClimber.getPIDController();
        rightClimberController.setFeedbackDevice(rightClimberEncoder);
        rightClimberController.setP(1);
        rightClimberController.setP(0);
        rightClimberController.setP(0);
        rightClimberController.setOutputRange(-.25,.25);

        m_LeftClimber.burnFlash();
        m_RightClimber.burnFlash();

        SmartDashboard.setDefaultBoolean("Run Left Climber Up", false);
        SmartDashboard.setDefaultBoolean("Run Right Climber Up", false);
    }

    public double leftClimberPos() {
        return leftClimberEncoder.getPosition();
    }

    public double rightClimberPos() {
        return leftClimberEncoder.getPosition();
    }

    @Override
    public void periodic(){
       SmartDashboard.getNumber("Left Climber Position", leftClimberEncoder.getPosition()); 
       SmartDashboard.getNumber("Right Climber Position", rightClimberEncoder.getPosition());
       
       SmartDashboard.putBoolean("Run Left Climber Up", false);
       SmartDashboard.putBoolean("Right Left Climber Up", false);
       
      // SmartDashboard.putBoolean("Run Left Climber Up",  false); 

       if (SmartDashboard.getBoolean("Run Left Climber Up", false)) {
        m_LeftClimber.set(.1);
       }
       else{
        m_LeftClimber.set(0);
       }

       if (SmartDashboard.getBoolean("Run Left Climber Down", false)) {
        m_LeftClimber.set(-.1);
       }
       else{
        m_LeftClimber.set(0);
       }

       if (SmartDashboard.getBoolean("Run Right Climber Up", false)) {
        m_RightClimber.set(.1);
       }
       else{
        m_RightClimber.set(0);
       }

       if (SmartDashboard.getBoolean("Run Right Climber Down", false)) {
        m_RightClimber.set(-.1);
       }
       else{
        m_RightClimber.set(0);
       }


       
      




    }


 
 
 
}
