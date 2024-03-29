// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.mechanisms.intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.TimingBudget;
import frc.robot.subsystems.mechanisms.MechanismConstants;

/** Add your docs here. */
public class IntakeIOSparkMax implements IntakeIO{
    private final CANSparkMax m_Wrist;
    private final CANSparkMax  m_Intake;
    private final AbsoluteEncoder wristEncoder;
    private final RelativeEncoder intakeEncoder;
    private final SparkPIDController wristController;
    private final SparkPIDController intakeController;
    private final PWMSparkMax blinken;

    private LaserCan laserCAN;


    public IntakeIOSparkMax() {
        m_Wrist = new CANSparkMax(MechanismConstants.kWristCanId, MotorType.kBrushless);
        m_Intake = new CANSparkMax(MechanismConstants.kIntakeCanId, MotorType.kBrushless);
        blinken = new PWMSparkMax(9);

        wristEncoder = m_Wrist.getAbsoluteEncoder(Type.kDutyCycle);
        intakeEncoder =  m_Intake.getEncoder();
        
        wristEncoder.setInverted(false);    
        wristEncoder.setPositionConversionFactor(360);
    
        m_Wrist.setIdleMode(IdleMode.kCoast);       //MOTORBRAKE
        m_Intake.setIdleMode(IdleMode.kCoast);      //MOTORBRAKE
        m_Wrist.setInverted(false);
        m_Intake.setInverted(true);
        m_Wrist.setSmartCurrentLimit(15);
        m_Intake.setSmartCurrentLimit(30);
    
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
        wristController.setP(.00028);
        wristController.setI(0);
        wristController.setD(0);
        wristController.setIZone(0);
        wristController.setFF(0.0002);
        wristController.setOutputRange(-0.70,0.70);
      //  wristController.setPositionPIDWrappingEnabled(false);

        int smartMotionSlot = 0;
        wristController.setSmartMotionMaxVelocity(1200, smartMotionSlot);
        wristController.setSmartMotionMinOutputVelocity(0, smartMotionSlot);
        wristController.setSmartMotionMaxAccel(700, smartMotionSlot);
        wristController.setSmartMotionAllowedClosedLoopError(1, smartMotionSlot);


        m_Wrist.burnFlash();
        m_Intake.burnFlash();

        laserCAN = new LaserCan(MechanismConstants.kLaserCanId);

        try {
            laserCAN.setRangingMode(RangingMode.SHORT);
            laserCAN.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
        } catch (ConfigurationFailedException e) {
        e.printStackTrace();
        }

 
    } 
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeRPM = intakeEncoder.getVelocity();
        inputs.wristposition = wristEncoder.getPosition();
        inputs.intakeCurrent = m_Intake.getOutputCurrent();
        inputs.wristCurrent = m_Wrist.getOutputCurrent();
        inputs.lcMeasurement =  laserCAN.getMeasurement() == null ? 0 : laserCAN.getMeasurement().distance_mm;
        
        
    }
    @Override
    public void setIntakeMotors(double intakeRPM, double wristPostion) {
        intakeController.setReference(intakeRPM, CANSparkMax.ControlType.kVelocity);
        wristController.setReference(wristPostion, CANSparkMax.ControlType.kSmartMotion);
    }
 
    @Override
    public void setBlinken(double blinkenValue){
        blinken.set(blinkenValue);
    }


    public double getMeasurement() {
        return laserCAN.getMeasurement() == null ? 0 : laserCAN.getMeasurement().distance_mm;
    }

    
    @Override
    public void stop() {
        m_Wrist.stopMotor();
        m_Intake.stopMotor();
     }
}
