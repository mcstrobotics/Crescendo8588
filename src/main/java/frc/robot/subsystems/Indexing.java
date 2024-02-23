package frc.robot.subsystems;

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
// CONSTANTS
import frc.robot.Constants.IntakeConstants; // TODO swap out intake constants for indexing constants and make according constants - Mihir
import frc.robot.Constants.IndexingConstants;

// TODO add javadocs if u want - Mihir

public class Indexing extends SubsystemBase {
  private boolean loaded = false;

  private CANSparkMax m_left;
  private CANSparkMax m_right;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine;

  public Indexing() {
    m_left = new CANSparkMax(IndexingConstants.kLeftCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_right = new CANSparkMax(IndexingConstants.kRightCanId, CANSparkLowLevel.MotorType.kBrushless);

    // Additional initialization stuff here if needed
    setCoast();
    
    // Creates a SysIdRoutine
    routine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(this::voltageIndexing, 
          log -> {
          log.motor("indexing")
              .voltage(
                  m_appliedVoltage.mut_replace(
                    m_left.get() * RobotController.getBatteryVoltage(), Volts))
              .linearPosition(m_distance.mut_replace(m_left.getEncoder().getPosition(), Meters))
              .linearVelocity(
                  m_velocity.mut_replace(m_left.getEncoder().getVelocity(), MetersPerSecond));
          },
      this
    ));
  }

  private void voltageIndexing(Measure<Voltage> volts){
    m_left.setVoltage(volts.in(Volts));
    m_right.setVoltage(-volts.in(Volts));
  }

  public void setBrake() {
    m_left.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_right.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setCoast() {
    m_left.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_right.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void indexIn() {
    // System.out.println("Intake in");
    SmartDashboard.putString("Index State", "In");
    SmartDashboard.putNumber("Index Left Velocity", -IntakeConstants.kIntakeInSpeed);
    SmartDashboard.putNumber("Index Right Velocity", IntakeConstants.kIntakeInSpeed);

    m_left.set(-IntakeConstants.kIntakeInSpeed);
    m_right.set(IntakeConstants.kIntakeInSpeed);
  }

  /** move index motors to push the note out */
  public void indexOut() {
    SmartDashboard.putString("Index State", "Out");
    SmartDashboard.putNumber("Index Left Velocity", IntakeConstants.kIntakeOutSpeed);
    SmartDashboard.putNumber("Index Right Velocity", -IntakeConstants.kIntakeOutSpeed);

    m_left.set(IntakeConstants.kIntakeOutSpeed);
    m_right.set(-IntakeConstants.kIntakeOutSpeed);
  }

  public void indexStop() {
    SmartDashboard.putString("Index State", "Stopped");
    SmartDashboard.putNumber("Index Left Velocity", 0);
    SmartDashboard.putNumber("Index Right Velocity", 0);
    
    m_right.set(0);
    m_left.set(0);
  }

  public void periodic() {
    // called once per run
  }

  public void setLoaded(boolean b) {
    loaded = b;
    SmartDashboard.putString("State", loaded ? "loaded" : "empty");
    System.out.println(loaded ? "loaded" : "empty");
  }

  public boolean isLoaded() {
    SmartDashboard.putString("State", loaded ? "loaded" : "empty");
    System.out.println(loaded ? "loaded" : "empty");
    return loaded;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }
}