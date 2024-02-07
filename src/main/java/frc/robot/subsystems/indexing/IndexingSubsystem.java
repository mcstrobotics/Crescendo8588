package frc.robot.subsystems.indexing;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants; // TODO swap out intake constants for indexing constants and make according constants - Mihir

// TODO, get rid of chassis file, and put all the logic hardcoded into variables into the constructor, and rename indexing subsystem to just "indexing" - Mihir
// right now in robotcontainer we call x = new IndexingSubsystem(new IndexingChassis(motor, motor))
// i want it so in robotcontainer we call x = new Indexing()

// TODO add javadocs if u want - Mihir

public class IndexingSubsystem extends SubsystemBase {
  // create private chassis variable
  private IndexingChassis chassis;

  public IndexingSubsystem(IndexingChassis chassis) {
    // initialize it
    this.chassis = chassis;

    // Additional initialization stuff here if needed
    setCoast();
  }

  public void setBrake() {
    // set idle mode for left index (look at intake)
    chassis.getLeftMotor().setIdleMode(CANSparkMax.IdleMode.kBrake);
    // set idle mode for right index (look at intake)
    chassis.getRightMotor().setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  public void setCoast() {
    // set coast mode for left index (look at intake)
    chassis.getLeftMotor().setIdleMode(CANSparkMax.IdleMode.kCoast);
    // set coast mode for right index (look at intake)
    chassis.getRightMotor().setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  public void indexIn() {
    // System.out.println("Intake in");
    SmartDashboard.putString("Index State", "In");
    SmartDashboard.putNumber("Index Left Velocity", -IntakeConstants.kIntakeInSpeed);
    SmartDashboard.putNumber("Index Right Velocity", IntakeConstants.kIntakeInSpeed);

    // print indexing in
    // set constant speed for index left (use intake constant)
    chassis.getLeftMotor().set(-IntakeConstants.kIntakeInSpeed);
    // set constant speed for index right (use intake constant)
    chassis.getRightMotor().set(IntakeConstants.kIntakeInSpeed);
  }

  /** move index motors to push the note out */
  public void indexOut() {
    // print indexing out
    // set constant speed for index left (use intake constant)
    // set constant speed for index right (use intake constant)
    // System.out.println("Intake out");
    // print indexing out
    // set constant speed for index left (use intake constant)
    // set constant speed for index right (use intake constant)
    SmartDashboard.putString("Index State", "Out");
    SmartDashboard.putNumber("Index Left Velocity", -IntakeConstants.kIntakeOutSpeed);
    SmartDashboard.putNumber("Index Right Velocity", IntakeConstants.kIntakeOutSpeed);

    chassis.getLeftMotor().set(-IntakeConstants.kIntakeOutSpeed);
    chassis.getRightMotor().set(IntakeConstants.kIntakeOutSpeed);
  }

  public void indexStop() {
    // System.out.println("Intake stopped");
    SmartDashboard.putString("Index State", "Stopped");
    SmartDashboard.putNumber("Index Left Velocity", 0);
    SmartDashboard.putNumber("Index Right Velocity", 0);
    chassis.getRightMotor().set(0);
    chassis.getLeftMotor().set(0);
  }

  public void periodic() {
    // called once per run
  }
}
