// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.Commands;

// CONSTANTS
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

// SUBSYSTEMS
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Shooter;
import frc.robot.usercontrol.GamepadF310;
// import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // TODO replace with CommandXboxController maybe - Mihir
  private GamepadF310 f310 = new GamepadF310(0);

  // making subsystem objects
  private final Intake m_intake = new Intake();
  private final Indexing m_indexing = new Indexing();
  private final Shooter m_shooter = new Shooter();

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
            m_robotDrive
        )
    );

    final Trigger A = new Trigger(f310::getA);
    final Trigger B = new Trigger(f310::getB);
    final Trigger X = new Trigger(f310::getX);
    final Trigger Y = new Trigger(f310::getY);

    // Intake Bindings (these are for temporary testing purposes, will change once
    // IntakeCommand is made / bindings will change)
    A.whileTrue(m_intake.run(m_intake::intakeIn));
    B.whileTrue(m_intake.run(m_intake::intakeOut));

    A.and(B.negate()).whileTrue(m_intake.run(m_intake::intakeIn));
    B.and(A.negate()).whileTrue(m_intake.run(m_intake::intakeOut));

    m_intake.setDefaultCommand(new RunCommand(m_intake::intakeStop, m_intake));

    // indexing
    X.whileTrue(m_indexing.run(m_indexing::indexIn));
    Y.whileTrue(m_indexing.run(m_indexing::indexOut));

    X.and(Y.negate()).whileTrue(m_indexing.run(m_indexing::indexIn));
    Y.and(X.negate()).whileTrue(m_indexing.run(m_indexing::indexOut));

    m_indexing.setDefaultCommand(new RunCommand(m_indexing::indexStop, m_indexing));

    // shooter
    A.whileTrue(m_shooter.run(m_shooter::shooterIn));
    B.whileTrue(m_shooter.run(m_shooter::shooterOut));
    // X.whileTrue(shooterSubsystem.run(shooterSubsystem::aimUp));
    // Y.whileTrue(shooterSubsystem.run(shooterSubsystem::aimDown));

    A.and(B.negate()).whileTrue(m_shooter.run(m_shooter::shooterIn));
    B.and(A.negate()).whileTrue(m_shooter.run(m_shooter::shooterOut));
    // X.and(Y.negate()).whileTrue(shooterSubsystem.run(shooterSubsystem::aimUp));
    // Y.and(X.negate()).whileTrue(shooterSubsystem.run(shooterSubsystem::aimDown));

    m_shooter.setDefaultCommand(new RunCommand(m_shooter::shooterStop, m_shooter));
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
