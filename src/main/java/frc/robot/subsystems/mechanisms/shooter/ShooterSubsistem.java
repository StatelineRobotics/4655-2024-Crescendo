// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.subsystems.mechanisms.MechanismConstants;


/** Add your docs here. */
public class ShooterSubsistem  extends SubsystemBase {
    private CANSparkMax m_TopShooter;
    private CANSparkMax  m_BottomShooter;
    private RelativeEncoder topShooterEncoder;
    private RelativeEncoder bottomShooterEncoder;
    private SparkPIDController topShooterController;
    private SparkPIDController bottomShooterController;

    public ShooterSubsistem(){

         m_BottomShooter = new CANSparkMax(MechanismConstants.kBottomShooterCanId, MotorType.kBrushless);
         m_TopShooter = new CANSparkMax(MechanismConstants.kTopShooterCanId, MotorType.kBrushless);

        bottomShooterEncoder =  m_BottomShooter.getEncoder();
        topShooterEncoder =  m_TopShooter.getEncoder();

         m_BottomShooter.restoreFactoryDefaults();
         m_BottomShooter.restoreFactoryDefaults();
         m_BottomShooter.setIdleMode(IdleMode.kBrake);
         m_BottomShooter.setIdleMode(IdleMode.kBrake);
         m_BottomShooter.setInverted(false);
         m_BottomShooter.setInverted(true);
         m_BottomShooter.setSmartCurrentLimit(30);
         m_BottomShooter.setSmartCurrentLimit(30);
        
        bottomShooterController = m_BottomShooter.getPIDController();
        bottomShooterController.setFeedbackDevice(bottomShooterEncoder);
        bottomShooterController.setP(1);
        bottomShooterController.setP(0);
        bottomShooterController.setP(0);
        bottomShooterController.setOutputRange(-.25,.25);

        topShooterController = m_TopShooter.getPIDController();
        topShooterController.setFeedbackDevice(topShooterEncoder);
        topShooterController.setP(1);
        topShooterController.setP(0);
        topShooterController.setP(0);
        topShooterController.setOutputRange(-.25,.25);

        m_BottomShooter.burnFlash();
        m_TopShooter.burnFlash();

    }

    public double bottomShooterPos() {
        return bottomShooterEncoder.getPosition();
    }

    public double topShooterPos() {
        return bottomShooterEncoder.getPosition();
    }

    @Override
    public void periodic(){
       SmartDashboard.putNumber("Left Climber Position", bottomShooterPos()); 
       SmartDashboard.putNumber("Left Climber Position", bottomShooterPos()); 

       






    }


 
 
 
}
