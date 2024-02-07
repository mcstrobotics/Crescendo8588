// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.usercontrol.GamepadF310;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.Commands;

// CONSTANTS
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IndexingConstants;
import frc.robot.Constants.ShooterConstants;

// SUBSYSTEMS
import frc.robot.subsystems.intake.IntakeChassis;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.indexing.IndexingChassis;
import frc.robot.subsystems.indexing.IndexingSubsystem;
import frc.robot.subsystems.shooter.ShooterChassis;
import frc.robot.subsystems.shooter.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private GamepadF310 f310 = new GamepadF310(0);

  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem(
      new IntakeChassis(
          new CANSparkMax(IntakeConstants.kIntakeCanId, CANSparkLowLevel.MotorType.kBrushless)));

  private IndexingSubsystem indexingSubsystem = new IndexingSubsystem(
      new IndexingChassis(
          new CANSparkMax(IndexingConstants.kLeftCanId, CANSparkLowLevel.MotorType.kBrushless),
          new CANSparkMax(IndexingConstants.kRightCanId, CANSparkLowLevel.MotorType.kBrushless)));

  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem(
      new ShooterChassis(
          new CANSparkMax(ShooterConstants.kBottomCanId, CANSparkLowLevel.MotorType.kBrushless),
          new CANSparkMax(ShooterConstants.kTopCanId, CANSparkLowLevel.MotorType.kBrushless),
          new CANSparkMax(ShooterConstants.kAimingCanId, CANSparkLowLevel.MotorType.kBrushless)));

  private AutonCommand autonCommand = new AutonCommand(m_robotDrive);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in 
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // swerve
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(() -> m_robotDrive.drive(
            -MathUtil.applyDeadband(f310.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(f310.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(f310.getRightX(), OIConstants.kDriveDeadband),
            true,
            true),
            m_robotDrive));

    final Trigger A = new Trigger(f310::getA);
    final Trigger B = new Trigger(f310::getB);
    final Trigger X = new Trigger(f310::getX);
    final Trigger Y = new Trigger(f310::getY);

    // Intake Bindings (these are for temporary testing purposes, will change once
    // IntakeCommand is made / bindings will change)
    A.whileTrue(intakeSubsystem.run(intakeSubsystem::intakeIn));
    B.whileTrue(intakeSubsystem.run(intakeSubsystem::intakeOut));

    A.and(B.negate()).whileTrue(intakeSubsystem.run(intakeSubsystem::intakeIn));
    B.and(A.negate()).whileTrue(intakeSubsystem.run(intakeSubsystem::intakeOut));

    intakeSubsystem.setDefaultCommand(new RunCommand(intakeSubsystem::intakeStop, intakeSubsystem));

    // INDEXING
    X.whileTrue(indexingSubsystem.run(indexingSubsystem::indexIn));
    Y.whileTrue(indexingSubsystem.run(indexingSubsystem::indexOut));

    X.and(Y.negate()).whileTrue(indexingSubsystem.run(indexingSubsystem::indexIn));
    Y.and(X.negate()).whileTrue(indexingSubsystem.run(indexingSubsystem::indexOut));

    indexingSubsystem.setDefaultCommand(new RunCommand(indexingSubsystem::indexStop, indexingSubsystem));

    // SHOOTER
    A.whileTrue(shooterSubsystem.run(shooterSubsystem::shooterIn));
    B.whileTrue(shooterSubsystem.run(shooterSubsystem::shooterOut));
    // X.whileTrue(shooterSubsystem.run(shooterSubsystem::aimUp));
    // Y.whileTrue(shooterSubsystem.run(shooterSubsystem::aimDown));

    A.and(B.negate()).whileTrue(shooterSubsystem.run(shooterSubsystem::shooterIn));
    B.and(A.negate()).whileTrue(shooterSubsystem.run(shooterSubsystem::shooterOut));
    // X.and(Y.negate()).whileTrue(shooterSubsystem.run(shooterSubsystem::aimUp));
    // Y.and(X.negate()).whileTrue(shooterSubsystem.run(shooterSubsystem::aimDown));

    shooterSubsystem.setDefaultCommand(new RunCommand(shooterSubsystem::shooterStop, shooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public AutonCommand getAutonCommand() {
    // autonCommand will run in autonomous
    return autonCommand;
  }
}
