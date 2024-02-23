package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

// TODO add javadocs if u want - Mihir

public class Shooter extends SubsystemBase {
  private CANSparkMax m_aim;
  private CANSparkMax m_bottom;
  private CANSparkMax m_top;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine;
  
  public Shooter() {
    this.m_aim = new CANSparkMax(ShooterConstants.kAimingCanId, CANSparkLowLevel.MotorType.kBrushless);
    this.m_bottom = new CANSparkMax(ShooterConstants.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);
    this.m_top = new CANSparkMax(ShooterConstants.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);
    setAimBrake();
    setShooterCoast();
    
    // Creates a SysIdRoutine
    routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::voltageAim, 
          log -> {
          log.motor("aim")
              .voltage(
                  m_appliedVoltage.mut_replace(
                    m_aim.get() * RobotController.getBatteryVoltage(), Volts))
              .linearPosition(m_distance.mut_replace(m_aim.getEncoder().getPosition(), Meters))
              .linearVelocity(
                  m_velocity.mut_replace(m_aim.getEncoder().getVelocity(), MetersPerSecond));
          },
      this
    ));

    // // Creates a SysIdRoutine
    // routine = new SysIdRoutine(
    //   new SysIdRoutine.Config(),
    //   new SysIdRoutine.Mechanism(this::voltageShoot, 
    //       log -> {
    //       log.motor("shoot")
    //           .voltage(
    //               m_appliedVoltage.mut_replace(
    //                 m_top.get() * RobotController.getBatteryVoltage(), Volts))
    //           .linearPosition(m_distance.mut_replace(m_top.getEncoder().getPosition(), Meters))
    //           .linearVelocity(
    //               m_velocity.mut_replace(m_top.getEncoder().getVelocity(), MetersPerSecond));
    //       },
    //   this
    // ));
  }

  private void voltageAim(Measure<Voltage> volts){
    m_aim.setVoltage(volts.in(Volts));
  }

  private void voltageShoot(Measure<Voltage> volts){
    m_bottom.setVoltage(volts.in(Volts));
    m_top.setVoltage(-volts.in(Volts));
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return routine.dynamic(direction);
  }
}
