// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;

// CONSTANTS
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private IntakeChassis chassis;
  private IntakeInputs inputs;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(IntakeChassis chassis, IntakeInputs inputs) {
    this.chassis = chassis;
    this.inputs = inputs;

    // Additional initialization stuff here if needed

    setCoast();
  }

  /** sets intake idlemode to brake */
  public void setBrake() {
    chassis.getIntake().setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  /** sets intake idlemode to coast */
  public void setCoast() {
    chassis.getIntake().setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  /** move intake motor to suck the note in */
  public void intakeIn() {
    System.out.println("Intake in");
    chassis.getIntake().set(IntakeConstants.kIntakeInSpeed);
  }

  /** move intake motor to push the note out */
  public void intakeOut() {
    System.out.println("Intake out");
    chassis.getIntake().set(-IntakeConstants.kIntakeOutSpeed);
  }

  /** stop intake motor */
  public void stopIntake() {
    System.out.println("Intake stopped");
    chassis.getIntake().set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
