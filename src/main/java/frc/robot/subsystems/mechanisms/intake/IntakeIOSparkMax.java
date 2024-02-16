package frc.robot.subsystems.mechanisms.intake;

import frc.robot.subsystems.mechanisms.MechanismConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;


public class IntakeIOSparkMax implements IntakeIO {
  private CANSparkMax m_IntakeSparkMax;
  private CANSparkMax  m_WristSparkMax;
  
  private RelativeEncoder intakeEncoder;
  private AbsoluteEncoder wristEncoder;

  private SparkPIDController intakeController;
  private SparkPIDController wristController;


  public IntakeIOSparkMax() {
    m_IntakeSparkMax = new CANSparkMax(MechanismConstants.kIntakeCanId, CANSparkMax.MotorType.kBrushless);
    m_WristSparkMax = new CANSparkMax(MechanismConstants.kWristCanId, CANSparkMax.MotorType.kBrushless);

    intakeEncoder = m_IntakeSparkMax.getEncoder();
    wristEncoder = m_WristSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

    m_IntakeSparkMax.restoreFactoryDefaults();
    m_WristSparkMax.restoreFactoryDefaults();

    m_IntakeSparkMax.setInverted(MechanismConstants.kIntakeSparkMaxInverted);
    m_WristSparkMax.setInverted(MechanismConstants.kIntakeSparkMaxInverted);
    
    m_IntakeSparkMax.setSmartCurrentLimit(30);
    m_WristSparkMax.setSmartCurrentLimit(30);

    m_IntakeSparkMax.enableVoltageCompensation(12.0);
    m_WristSparkMax.enableVoltageCompensation(12.0);

    m_IntakeSparkMax.setIdleMode(IdleMode.kBrake);
    m_WristSparkMax.setIdleMode(IdleMode.kBrake);

    // rotations, rps

    intakeController = m_IntakeSparkMax.getPIDController();
    intakeController.setFeedbackDevice(intakeEncoder);
    intakeController.setP(MechanismConstants.kIntakeP);
    intakeController.setP(MechanismConstants.kIntakeP);
    intakeController.setP(MechanismConstants.kIntakeP);
    intakeController.setOutputRange(MechanismConstants.kIntakeMinOutput,
      MechanismConstants.kIntakeMaxOutput);

    
    wristController = m_WristSparkMax.getPIDController();
    wristController.setFeedbackDevice(wristEncoder);
    intakeController.setP(MechanismConstants.kWristP);
    intakeController.setP(MechanismConstants.kWristP);
    intakeController.setP(MechanismConstants.kWristP);
    wristController.setOutputRange(MechanismConstants.kWristMinOutput,
      MechanismConstants.kWristMaxOutput);

    m_IntakeSparkMax.burnFlash();
    m_WristSparkMax.burnFlash();
  
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelocityRPM = intakeEncoder.getVelocity();
    inputs.intakeAppliedVolts = m_IntakeSparkMax.getAppliedOutput();
    inputs.intakeOutputCurrent = m_IntakeSparkMax.getOutputCurrent();

    inputs.wristPosition = wristEncoder.getPosition();
    inputs.wristAppliedVolts = m_WristSparkMax.getAppliedOutput();
    inputs.wristOutputCurrent = m_WristSparkMax.getOutputCurrent();

  }

  @Override
  public void setIntakeRPM(double rpm) {
    intakeController.setReference(
        rpm,
        CANSparkBase.ControlType.kVelocity);
  }

  @Override
  public void setwristPostion(double angle) {
    wristController.setReference(
        angle,
        CANSparkBase.ControlType.kDutyCycle);
  }

  @Override
  public void setIntakeBrakeMode(boolean enabled) {
    m_IntakeSparkMax.setIdleMode(enabled ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);
  }


  @Override
  public void stop() {
    m_IntakeSparkMax.set(0);
    m_IntakeSparkMax.set(0);
  }
}
