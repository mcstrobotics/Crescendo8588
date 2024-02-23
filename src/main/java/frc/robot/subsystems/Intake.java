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
import com.revrobotics.CANSparkLowLevel;

// CONSTANTS
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private CANSparkMax m_intake;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine;

  public Intake() {
    this.m_intake = new CANSparkMax(IntakeConstants.kIntakeCanId, CANSparkLowLevel.MotorType.kBrushless);

    // Additional initialization stuff here if needed

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
  public void intakeIn() {
    // System.out.println("Intake in");
    SmartDashboard.putString("Intake State", "In");
    SmartDashboard.putNumber("Intake Velocity", IntakeConstants.kIntakeInSpeed);

    m_intake.set(IntakeConstants.kIntakeInSpeed);
  }

  /** move intake motor to push the note out */
  public void intakeOut() {
    // System.out.println("Intake out");
    SmartDashboard.putString("Intake State", "Out");
    SmartDashboard.putNumber("Intake Velocity", -IntakeConstants.kIntakeOutSpeed);

    m_intake.set(-IntakeConstants.kIntakeOutSpeed);
  }

  /** stop intake motor */
  public void intakeStop() {
    // System.out.println("Intake stopped");
    SmartDashboard.putString("Intake State", "Stopped");
    SmartDashboard.putNumber("Intake Velocity", 0);

    m_intake.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return routine.dynamic(direction);
  }
}
