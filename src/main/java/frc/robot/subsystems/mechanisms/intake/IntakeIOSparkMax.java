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


//LC import au.grapplerobotics.ConfigurationFailedException;
//LC import au.grapplerobotics.LaserCan;

import frc.robot.subsystems.mechanisms.MechanismConstants;

/** Add your docs here. */
public class IntakeIOSparkMax implements IntakeIO{
    private final CANSparkMax m_Wrist;
    private final CANSparkMax  m_Intake;
    private final AbsoluteEncoder wristEncoder;
    private final RelativeEncoder intakeEncoder;
    private final SparkPIDController wristController;
    private final SparkPIDController intakeController;
//LC    private final LaserCan lasercan;  

    public IntakeIOSparkMax() {
        m_Wrist = new CANSparkMax(MechanismConstants.kWristCanId, MotorType.kBrushless);
        m_Intake = new CANSparkMax(MechanismConstants.kIntakeCanId, MotorType.kBrushless);
    

        wristEncoder = m_Wrist.getAbsoluteEncoder(Type.kDutyCycle);
        intakeEncoder =  m_Intake.getEncoder();
        
        wristEncoder.setInverted(false);    
        wristEncoder.setPositionConversionFactor(360);
    
        m_Wrist.setIdleMode(IdleMode.kCoast);       //MOTORBRAKE
        m_Intake.setIdleMode(IdleMode.kCoast);      //MOTORBRAKE
        m_Wrist.setInverted(false);
        m_Intake.setInverted(true);
        m_Wrist.setSmartCurrentLimit(20);
        m_Intake.setSmartCurrentLimit(20);
    
        intakeController = m_Intake.getPIDController();
        intakeController.setFeedbackDevice(intakeEncoder);
        intakeController.setP(.000008);
        intakeController.setI(0);
        intakeController.setD(0);
        intakeController.setIZone(0);
        intakeController.setFF(.0007);
        intakeController.setOutputRange(-.40,.40);

        wristController = m_Wrist.getPIDController();
        wristController.setFeedbackDevice(wristEncoder);
        wristController.setP(.0007);
        wristController.setI(0);
        wristController.setD(0);
        wristController.setIZone(0);
        wristController.setFF(0);
        wristController.setOutputRange(-0.50,0.50);
        wristController.setPositionPIDWrappingEnabled(false);

        int smartMotionSlot = 0;
        wristController.setSmartMotionMaxVelocity(1000, smartMotionSlot);
        wristController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        wristController.setSmartMotionMaxAccel(500, smartMotionSlot);
        wristController.setSmartMotionAllowedClosedLoopError(1, smartMotionSlot);


        m_Wrist.burnFlash();
        m_Intake.burnFlash();

    

  //LC      lasercan = new LaserCan(MechanismConstants.kLaserCanId);
  //LC      try {
  //LC      lasercan.setRangingMode(LaserCan.RangingMode.SHORT);
  //LC      lasercan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
  //LC      lasercan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
  //LC      } catch (ConfigurationFailedException e) {
  //LC      System.out.println("Configuration failed! " + e);
  //LC      }
    } 
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeRPM = intakeEncoder.getVelocity();
        inputs.wristposition = wristEncoder.getPosition();
        inputs.intakeCurrent = m_Intake.getOutputCurrent();
        inputs.wristCurrent = m_Wrist.getOutputCurrent();
    
        
    }
    @Override
    public void setIntakeMotors(double intakeRPM, double wristPostion) {
        intakeController.setReference(intakeRPM, CANSparkMax.ControlType.kVelocity);
        wristController.setReference(wristPostion, CANSparkMax.ControlType.kSmartMotion);
    }
 
    @Override
    public void stop() {
        m_Wrist.stopMotor();
        m_Intake.stopMotor();
     }
}
