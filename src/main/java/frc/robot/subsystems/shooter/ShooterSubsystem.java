package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

// TODO add javadocs if u want - Mihir

public class ShooterSubsystem extends SubsystemBase {
  private ShooterChassis chassis;

  public ShooterSubsystem(ShooterChassis chassis) {
    this.chassis = chassis;
  }

  // Sets aim motor to brake mode
  public void setAimBrake() {
    chassis.getShooterAim().setIdleMode(CANSparkMax.IdleMode.kBrake);
  }
  
  // Set the aim motor to coast
  public void setAimCoast() {
    chassis.getShooterAim().setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  // Set the shooter motors to brake
  public void setShooterBrake() {
    chassis.getShooterBottom().setIdleMode(CANSparkMax.IdleMode.kBrake);
    chassis.getShooterTop().setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  // Set the shooter motors to coast
  public void setShooterCoast() {
    chassis.getShooterBottom().setIdleMode(CANSparkMax.IdleMode.kCoast);
    chassis.getShooterTop().setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  // Shoot
  public void shooterOut() {
    SmartDashboard.putString("Shooter State", "Out");
    SmartDashboard.putNumber("Shooter Bottom Velocity", ShooterConstants.kShooterBottomOutSpeed);
    SmartDashboard.putNumber("Shooter Top Velocity", -ShooterConstants.kShooterTopOutSpeed);

    chassis.getShooterBottom().set(ShooterConstants.kShooterBottomOutSpeed);
    chassis.getShooterTop().set(-ShooterConstants.kShooterTopOutSpeed);
  }

  // Retract Shooters
  public void shooterIn() {
    SmartDashboard.putString("Shooter State", "In");
    SmartDashboard.putNumber("Shooter Bottom Velocity", -ShooterConstants.kShooterBottomInSpeed);
    SmartDashboard.putNumber("Shooter Top Velocity", ShooterConstants.kShooterTopInSpeed);

    chassis.getShooterBottom().set(-ShooterConstants.kShooterBottomInSpeed);
    chassis.getShooterTop().set(ShooterConstants.kShooterTopInSpeed);
  }
  
  // Stop Shooters
  public void shooterStop() {
    SmartDashboard.putString("Shooter State", "Stopped");
    SmartDashboard.putNumber("Shooter Bottom Velocity", 0);
    SmartDashboard.putNumber("Shooter Top Velocity", 0);

    chassis.getShooterBottom().set(0);
    chassis.getShooterTop().set(0);
  }

  // public void shoot() {
  //   SmartDashboard.putString("pew pew", "pew");
  //   // don't need yet until index and shooter combined
  // }

  // public void aimUp() {
  //   float angle = 0; // encoder
  //   // set servo to angle +10
  // }

  // public void aimDown() {
  //   float angle = 0; // encoder
  //   // set servo to angle -10
  // }

  // public void aimStop() {
  //   SmartDashboard.putString("Aim State", "Stopped");
  //   SmartDashboard.putNumber("Aim Velocity", 0);

  //   chassis.getShooterAim().set(0);
  // }
}
