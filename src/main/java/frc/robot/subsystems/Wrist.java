// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.WristConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel;

/** A robot wrist subsystem that moves with a motion profile. */
public class Wrist extends TrapezoidProfileSubsystem {
  private final CANSparkMax m_motor = new CANSparkMax(WristConstants.kArmCanId, CANSparkLowLevel.MotorType.kBrushless);
  private final RelativeEncoder m_encoder;
  private final SparkPIDController m_PIDController = m_motor.getPIDController(); 

  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          WristConstants.kSVolts, WristConstants.kGVolts,
          WristConstants.kVVoltSecondPerRad, WristConstants.kAVoltSecondSquaredPerRad);

  /** Create a new Wrist. */
  public Wrist() {
    super(new TrapezoidProfile.Constraints(WristConstants.kMaxVelocityRadPerSecond, WristConstants.kMaxAccelerationRadPerSecSquared), WristConstants.kWristOffsetRads);
    
    m_encoder = m_motor.getAlternateEncoder(WristConstants.kCountsPerRev);
    m_PIDController.setP(1);
    m_PIDController.setI(0);
    m_PIDController.setD(0);
    
    m_PIDController.setFeedbackDevice(m_encoder);
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        
    // Set the setpoint for the PID controller
    m_PIDController.setReference(setpoint.position, CANSparkBase.ControlType.kPosition, 0, feedforward / 12.0);
  }

  public Command setWristGoalCommand(double kWristOffsetRads) {
    return Commands.runOnce(() -> setGoal(kWristOffsetRads), this);
  }
}
