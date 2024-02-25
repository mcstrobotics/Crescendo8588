// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel;

// CONSTANTS
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorContants;

public class Intake extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax m_intake;
  private RelativeEncoder m_intakeEncoder;
  private SparkPIDController m_intakePIDController;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine;

  public Intake() {
    this.m_intake = new CANSparkMax(IntakeConstants.kIntakeCanId, CANSparkLowLevel.MotorType.kBrushless);

    // Factory reset, so we get the SPARK MAX to a known state before configuring them. Useful in case a SPARK MAX is swapped out.
    m_intake.restoreFactoryDefaults();

    m_intakeEncoder = m_intake.getEncoder();
    
    // m_intakePIDController = m_intake.getPIDController();
    // m_intakePIDController.setFeedbackDevice(m_intakeEncoder);

    // m_intakeEncoder.setPositionConversionFactor(IntakeConstants.kBottomEncoderPositionFactor);
    // m_intakeEncoder.setVelocityConversionFactor(IntakeConstants.kBottomEncoderVelocityFactor);

    // m_intakePIDController.setP( IntakeConstants.kP);
    // m_intakePIDController.setI( IntakeConstants.kI);
    // m_intakePIDController.setD( IntakeConstants.kD);
    // m_intakePIDController.setFF(IntakeConstants.kFF);
    // // m_intakePIDController.setOutputRange(IntakeConstants.kIntakeMinOutput, IntakeConstants.kIntakeMaxOutput);
    // m_intakePIDController.setOutputRange(-1, 1);

    // setCoast();
    // m_intake.setSmartCurrentLimit(MotorContants.kMotorCurrentLimit);

    // // Save the SPARK MAX configurations. If a SPARK MAX browns out during operation, it will maintain the above configurations.
    // m_intake.burnFlash();

    // m_intakeEncoder.setPosition(0);

    setCoast();

    // Creates a SysIdRoutine
    routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::voltageIntake, 
          log -> {
          log.motor("intake")
              .voltage(
                  m_appliedVoltage.mut_replace(
                    m_intake.get() * RobotController.getBatteryVoltage(), Volts))
              .linearPosition(m_distance.mut_replace(m_intake.getEncoder().getPosition(), Meters))
              .linearVelocity(
                  m_velocity.mut_replace(m_intake.getEncoder().getVelocity(), MetersPerSecond));
          },
      this
    ));
  }

  private void voltageIntake(Measure<Voltage> volts){
    m_intake.setVoltage(volts.in(Volts));
  }

  /** sets intake idlemode to brake */
  public void setBrake() {
    m_intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /** sets intake idlemode to coast */
  public void setCoast() {
    m_intake.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  /** move intake motor to suck the note in */
  public void intake() {
    SmartDashboard.putString("Intake State", "intake");

    m_intake.set(.8);

    // m_intakePIDController.setReference(MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity);
  }

  /** stop intake motor */
  public void stop() {
    SmartDashboard.putString("Intake State", "stop");

    m_intake.set(0);

    // m_intakePIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
  }
  
  // /** move intake motor to push the note out */
  // public void intakeOut() {
  //   // System.out.println("Intake out");
  //   SmartDashboard.putString("Intake State", "Out");

  //   m_intake.set(-.8);

  //   // m_intakePIDController.setReference(-MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity);
  // }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Encoder Position", m_intakeEncoder.getPosition());
    SmartDashboard.putNumber("Intake Encoder Velocity", m_intakeEncoder.getVelocity());
    SmartDashboard.putNumber("Intake Temp", m_intake.getMotorTemperature());
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}
