// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.mechanisms.MechanismConstants;

/** Add your docs here. */
public class IntakeIOSparkMax implements IntakeIO{
    private final CANSparkMax m_Wrist;
    private final CANSparkMax  m_Intake;
    private final AbsoluteEncoder wristEncoder;
    private final RelativeEncoder intakeEncoder;
    private final SparkPIDController wristController;
    private final SparkPIDController intakeController;
    private final LaserCan lasercan;  

    public IntakeIOSparkMax() {
        m_Wrist = new CANSparkMax(MechanismConstants.kIntakeCanId, MotorType.kBrushless);
        m_Intake = new CANSparkMax(MechanismConstants.kWristCanId, MotorType.kBrushless);
    
        wristEncoder =  m_Wrist.getAbsoluteEncoder(Type.kDutyCycle);
        intakeEncoder =  m_Intake.getEncoder();
        wristEncoder.setInverted(false);
        wristEncoder.setPositionConversionFactor(360);

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

        lasercan = new LaserCan(MechanismConstants.kLaserCanId);
        try {
        lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
        lasercan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        lasercan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
        }
    } 
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeRPM = intakeEncoder.getVelocity();
        inputs.wristposition = wristEncoder.getPosition();
        SmartDashboard.putNumber("IntakeRPM", intakeEncoder.getVelocity()); 
        SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
        
    }
    @Override
    public void setIntakeMotors(double intakeRPM, double wristPostion) {
        intakeController.setReference(intakeRPM, CANSparkMax.ControlType.kVelocity);
        wristController.setReference(wristPostion, CANSparkMax.ControlType.kDutyCycle);
    }
 
    @Override
    public void stop() {
        m_Wrist.stopMotor();
        m_Intake.stopMotor();
     }
}
