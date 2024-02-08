package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.Command;

// TODO add javadocs if u want - Mihir

public class Shooter extends SubsystemBase {
  private CANSparkMax shooterAim;
  private CANSparkMax shooterBottom;
  private CANSparkMax shooterTop;

  public Shooter() {
    this.shooterAim = new CANSparkMax(ShooterConstants.kAimingCanId, CANSparkLowLevel.MotorType.kBrushless);
    this.shooterBottom = new CANSparkMax(ShooterConstants.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);
    this.shooterTop = new CANSparkMax(ShooterConstants.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    setAimBrake();
    setShooterCoast();
  }

  // Sets aim motor to brake mode
  public void setAimBrake() {
    shooterAim.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  // Set the aim motor to coast
  public void setAimCoast() {
    shooterAim.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  // Set the shooter motors to brake
  public void setShooterBrake() {
    shooterBottom.setIdleMode(CANSparkMax.IdleMode.kBrake);
    shooterTop.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  // Set the shooter motors to coast
  public void setShooterCoast() {
    shooterBottom.setIdleMode(CANSparkMax.IdleMode.kCoast);
    shooterTop.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  // Shoot
  public void shooterOut() {
    SmartDashboard.putString("Shooter State", "Out");
    SmartDashboard.putNumber("Shooter Bottom Velocity", ShooterConstants.kShooterBottomOutSpeed);
    SmartDashboard.putNumber("Shooter Top Velocity", -ShooterConstants.kShooterTopOutSpeed);

    shooterBottom.set(ShooterConstants.kShooterBottomOutSpeed);
    shooterTop.set(-ShooterConstants.kShooterTopOutSpeed);
  }

  // Retract Shooters
  public void shooterIn() {
    SmartDashboard.putString("Shooter State", "In");
    SmartDashboard.putNumber("Shooter Bottom Velocity", -ShooterConstants.kShooterBottomInSpeed);
    SmartDashboard.putNumber("Shooter Top Velocity", ShooterConstants.kShooterTopInSpeed);

    shooterBottom.set(-ShooterConstants.kShooterBottomInSpeed);
    shooterTop.set(ShooterConstants.kShooterTopInSpeed);
  }

  // Stop Shooters
  public void shooterStop() {
    SmartDashboard.putString("Shooter State", "Stopped");
    SmartDashboard.putNumber("Shooter Bottom Velocity", 0);
    SmartDashboard.putNumber("Shooter Top Velocity", 0);

    shooterBottom.set(0);
    shooterTop.set(0);
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
