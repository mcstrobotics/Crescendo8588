package frc.robot.subsystems;

import frc.robot.Constants.MotorContants;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
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

  private RelativeEncoder m_bottomEncoder;
  private RelativeEncoder m_topEncoder;

  private SparkPIDController m_bottomPIDController;
  private SparkPIDController m_topPIDController;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine;
  
  public Shooter() {
    this.m_aim = new CANSparkMax(ShooterConstants.kAimingCanId, CANSparkLowLevel.MotorType.kBrushless);

    m_bottom = new CANSparkMax(ShooterConstants.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless);
    m_top = new CANSparkMax(ShooterConstants.kTopCanId, CANSparkLowLevel.MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring them. Useful in case a SPARK MAX is swapped out.
    m_bottom.restoreFactoryDefaults();
    m_top.restoreFactoryDefaults();

    m_bottom.setInverted(false);
    m_top.setInverted(true);

    // Setup encoders and PID controllers
    m_bottomEncoder = m_bottom.getEncoder();
    m_topEncoder = m_top.getEncoder();

    // m_bottomPIDController = m_bottom.getPIDController();
    // m_bottomPIDController.setFeedbackDevice(m_bottomEncoder);
    // m_topPIDController = m_top.getPIDController();
    // m_topPIDController.setFeedbackDevice(m_topEncoder);

    // m_bottomEncoder.setPositionConversionFactor(ShooterConstants.kBottomEncoderPositionFactor);
    // m_bottomEncoder.setVelocityConversionFactor(ShooterConstants.kBottomEncoderVelocityFactor);
    // m_topEncoder.setPositionConversionFactor(ShooterConstants.kTopEncoderPositionFactor);
    // m_topEncoder.setVelocityConversionFactor(ShooterConstants.kTopEncoderVelocityFactor);

    // m_bottomPIDController.setP( ShooterConstants.kBottomP);
    // m_bottomPIDController.setI( ShooterConstants.kBottomI);
    // m_bottomPIDController.setD( ShooterConstants.kBottomD);
    // m_bottomPIDController.setFF(ShooterConstants.kBottomFF);
    // m_bottomPIDController.setOutputRange(-1, 1);
    
    // m_topPIDController.setP( ShooterConstants.kTopP);
    // m_topPIDController.setI( ShooterConstants.kTopI);
    // m_topPIDController.setD( ShooterConstants.kTopD);
    // m_topPIDController.setFF(ShooterConstants.kTopFF);
    // m_topPIDController.setOutputRange(-1, 1);

    // setCoast();
    // m_bottom.setSmartCurrentLimit(MotorContants.kMotorCurrentLimit);
    // m_top.setSmartCurrentLimit(MotorContants.kMotorCurrentLimit);

    // // Save the SPARK MAX configurations. If a SPARK MAX browns out during operation, it will maintain the above configurations.
    // m_bottom.burnFlash();
    // m_top.burnFlash();

    // m_bottomEncoder.setPosition(0);
    // m_topEncoder.setPosition(0);   
    
    // setAimBrake();
    // setShooterCoast();
    
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
    //       log.motor("shoot-top")
    //           .voltage(
    //               m_appliedVoltage.mut_replace(
    //                 m_top.get() * RobotController.getBatteryVoltage(), Volts))
    //           .linearPosition(m_distance.mut_replace(m_top.getEncoder().getPosition(), Meters))
    //           .linearVelocity(
    //               m_velocity.mut_replace(m_top.getEncoder().getVelocity(), MetersPerSecond));
    //       },
    //   this
    // ));

    // // Creates a SysIdRoutine
    // routine = new SysIdRoutine(
    //   new SysIdRoutine.Config(),
    //   new SysIdRoutine.Mechanism(this::voltageShoot, 
    //       log -> {
    //       log.motor("shoot-bottom")
    //           .voltage(
    //               m_appliedVoltage.mut_replace(
    //                 m_bottom.get() * RobotController.getBatteryVoltage(), Volts))
    //           .linearPosition(m_distance.mut_replace(m_bottom.getEncoder().getPosition(), Meters))
    //           .linearVelocity(
    //               m_velocity.mut_replace(m_bottom.getEncoder().getVelocity(), MetersPerSecond));
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

  // // Sets aim motor to brake mode
  // public void setAimBrake() {
  //   m_aim.setIdleMode(CANSparkMax.IdleMode.kBrake);
  // }

  // // Set the aim motor to coast
  // public void setAimCoast() {
  //   m_aim.setIdleMode(CANSparkMax.IdleMode.kCoast);
  // }

  // Set the shooter motors to brake
  public void setBrake() {
    m_bottom.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_top.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  // Set the shooter motors to coast
  public void setCoast() {
    m_bottom.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_top.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  // Shoot
  public void shoot() {
    SmartDashboard.putString("Shooter State", "shoot");

    m_bottom.set(.8);
    m_top.set(.8);

    // m_bottomPIDController.setReference(MotorContants.kShootingSpeed, CANSparkMax.ControlType.kVelocity);
    // m_topPIDController.setReference(MotorContants.kShootingSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public void intake() {
    SmartDashboard.putString("Shooter State", "intake");

    m_bottom.set(.8);
    m_top.set(.8);
    
    // m_bottomPIDController.setReference(MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity);
    // m_topPIDController.setReference(MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity);
  }

  public void shooterIntake() {
    SmartDashboard.putString("Shooter State", "shooter-intake");

    m_bottom.set(-.8);
    m_top.set(-.8);
    
    // m_bottomPIDController.setReference(-MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity);
    // m_topPIDController.setReference(-MotorContants.kIntakeSpeed, CANSparkMax.ControlType.kVelocity);
  }

  // Stop Shooters
  public void stop() {
    SmartDashboard.putString("Shooter State", "stopped");

    m_bottom.set(0);
    m_top.set(0);

    // m_bottomPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
    // m_topPIDController.setReference(0, CANSparkMax.ControlType.kVelocity);
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

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Top Encoder Position", m_topEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Top Encoder Velocity", m_topEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Top Temp", m_top.getMotorTemperature());

    SmartDashboard.putNumber("Shooter Bottom Encoder Position", m_bottomEncoder.getPosition());
    SmartDashboard.putNumber("Shooter Bottom Encoder Velocity", m_bottomEncoder.getVelocity());
    SmartDashboard.putNumber("Shooter Bottom Temp", m_bottom.getMotorTemperature());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
      return routine.dynamic(direction);
  }
}
