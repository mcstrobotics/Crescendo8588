package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.Command;

// TODO add javadocs if u want - Mihir

public class Shooter extends SubsystemBase {
  private CANSparkMax m_aim;
  private CANSparkMax m_bottom;
  private CANSparkMax m_top;

  public Shooter() {
    this.m_aim = new CANSparkMax(ShooterConstants.kAimingCanId, CANSparkLowLevel.MotorType.kBrushless);
    this.m_bottom = new CANSparkMax(ShooterConstants.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);
    this.m_top = new CANSparkMax(ShooterConstants.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    setAimBrake();
    setShooterCoast();
  }

  // Sets aim motor to brake mode
  public void setAimBrake() {
    m_aim.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  // Set the aim motor to coast
  public void setAimCoast() {
    m_aim.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  // Set the shooter motors to brake
  public void setShooterBrake() {
    m_bottom.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_top.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  // Set the shooter motors to coast
  public void setShooterCoast() {
    m_bottom.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_top.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  // Shoot
  public void shooterOut() {
    SmartDashboard.putString("Shooter State", "Out");
    SmartDashboard.putNumber("Shooter Bottom Velocity", ShooterConstants.kShooterBottomOutSpeed);
    SmartDashboard.putNumber("Shooter Top Velocity", -ShooterConstants.kShooterTopOutSpeed);

    m_bottom.set(ShooterConstants.kShooterBottomOutSpeed);
    m_top.set(-ShooterConstants.kShooterTopOutSpeed);
  }

  // Retract Shooters
  public void shooterIn() {
    SmartDashboard.putString("Shooter State", "In");
    SmartDashboard.putNumber("Shooter Bottom Velocity", -ShooterConstants.kShooterBottomInSpeed);
    SmartDashboard.putNumber("Shooter Top Velocity", ShooterConstants.kShooterTopInSpeed);

    m_bottom.set(-ShooterConstants.kShooterBottomInSpeed);
    m_top.set(ShooterConstants.kShooterTopInSpeed);
  }

  // Stop Shooters
  public void shooterStop() {
    SmartDashboard.putString("Shooter State", "Stopped");
    SmartDashboard.putNumber("Shooter Bottom Velocity", 0);
    SmartDashboard.putNumber("Shooter Top Velocity", 0);

    m_bottom.set(0);
    m_top.set(0);
  }

  // public void shoot() {
  // SmartDashboard.putString("pew pew", "pew");
  // // don't need yet until index and shooter combined
  // }

  // public void aimUp() {
  // float angle = 0; // encoder
  // // set servo to angle +10
  // }

  // public void aimDown() {
  // float angle = 0; // encoder
  // // set servo to angle -10
  // }

  // public void aimStop() {
  // SmartDashboard.putString("Aim State", "Stopped");
  // SmartDashboard.putNumber("Aim Velocity", 0);

  // shooterAim.set(0);
  // }
}
