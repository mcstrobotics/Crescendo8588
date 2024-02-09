package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.Command;

// CONSTANTS
import frc.robot.Constants.IntakeConstants; // TODO swap out intake constants for indexing constants and make according constants - Mihir
import frc.robot.Constants.IndexingConstants;

// TODO add javadocs if u want - Mihir

public class Indexing extends SubsystemBase {
  private CANSparkMax m_left;
  private CANSparkMax m_right;

  public Indexing() {
    m_left = new CANSparkMax(IndexingConstants.kLeftCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_right = new CANSparkMax(IndexingConstants.kRightCanId, CANSparkLowLevel.MotorType.kBrushless);

    // Additional initialization stuff here if needed
    setCoast();
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
}