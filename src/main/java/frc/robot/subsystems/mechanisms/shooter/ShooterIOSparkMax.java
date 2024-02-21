// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import frc.robot.subsystems.mechanisms.MechanismConstants;


/** Add your docs here. */
public class ShooterIOSparkMax implements ShooterIO {
    private CANSparkMax m_TopShooter;
    private CANSparkMax  m_BottomShooter;
    private RelativeEncoder topShooterEncoder;
    private RelativeEncoder bottomShooterEncoder;
    private SparkPIDController topShooterController;
    private SparkPIDController bottomShooterController;

    public ShooterIOSparkMax(){
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

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.TopShooterRPM = topShooterEncoder.getVelocity();
        inputs.BottomShooterRPM = bottomShooterEncoder.getVelocity();
        SmartDashboard.putNumber("Top Shooter RPM", topShooterEncoder.getVelocity()); 
        SmartDashboard.putNumber("Bottom Shooter RPM", bottomShooterEncoder.getVelocity());

        
    }

    @Override
    public void setShooterMotors(double shooterTopRPM, double shooterBottomRPM) {
        topShooterController.setReference(shooterTopRPM, CANSparkMax.ControlType.kVelocity);
        bottomShooterController.setReference(shooterBottomRPM, CANSparkMax.ControlType.kVelocity);
    }
 
    @Override
    public void stop() {
        m_TopShooter.stopMotor();
        m_BottomShooter.stopMotor();
     }

}


