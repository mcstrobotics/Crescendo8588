// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.Commands;

// CONSTANTS
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutonCommand;

// SUBSYSTEMS
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Shooter;
//import frc.robot.usercontrol.GamepadF310;

// YAGSL Imports
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.io.File;
// import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // OLD Swerve stuff
  //private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private AutonCommand autonCommand = new AutonCommand(m_robotDrive);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "/swerve/maxswerve/"));

  // replaced with CommandXboxController
  // private GamepadF310 f310 = new GamepadF310(0);
  final CommandXboxController driverXbox = new CommandXboxController(0);

  // making subsystem objects
  private final Intake m_intake = new Intake();
  private final Indexing m_indexing = new Indexing();
  private final Shooter m_shooter = new Shooter();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   driverXbox.getHID()::getYButtonPressed,
                                                                   driverXbox.getHID()::getAButtonPressed,
                                                                   driverXbox.getHID()::getXButtonPressed,
                                                                   driverXbox.getHID()::getBButtonPressed);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX(),
        () -> -driverXbox.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    // OLD SWERVE DRIVE BINDINGS
    // m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
    //     new RunCommand(() -> m_robotDrive.drive(
    //         -MathUtil.applyDeadband(f310.getLeftY(), OIConstants.kDriveDeadband),
    //         -MathUtil.applyDeadband(f310.getLeftX(), OIConstants.kDriveDeadband),
    //         -MathUtil.applyDeadband(f310.getRightX(), OIConstants.kDriveDeadband),
    //         true,
    //         true),
    //         m_robotDrive
    //     )
    // );

    // final Trigger A = new Trigger(f310::getA);
    // final Trigger B = new Trigger(f310::getB);
    // final Trigger X = new Trigger(f310::getX);
    // final Trigger Y = new Trigger(f310::getY);

    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));

    //B.onTrue(intakeNoteCommand);

    // // Intake Bindings (these are for temporary testing purposes, will change once
    // // IntakeCommand is made / bindings will change)
    // A.whileTrue(m_intake.run(m_intake::intakeIn));
    // B.whileTrue(m_intake.run(m_intake::intakeOut));

    // A.and(B.negate()).whileTrue(m_intake.run(m_intake::intakeIn));
    // B.and(A.negate()).whileTrue(m_intake.run(m_intake::intakeOut));

    // m_intake.setDefaultCommand(new RunCommand(m_intake::intakeStop, m_intake));

    // // indexing
    // X.whileTrue(m_indexing.run(m_indexing::indexIn));
    // Y.whileTrue(m_indexing.run(m_indexing::indexOut));

    // X.and(Y.negate()).whileTrue(m_indexing.run(m_indexing::indexIn));
    // Y.and(X.negate()).whileTrue(m_indexing.run(m_indexing::indexOut));

    // m_indexing.setDefaultCommand(new RunCommand(m_indexing::indexStop, m_indexing));

    // // shooter
    // A.whileTrue(m_shooter.run(m_shooter::shooterIn));
    // B.whileTrue(m_shooter.run(m_shooter::shooterOut));
    // // X.whileTrue(shooterSubsystem.run(shooterSubsystem::aimUp));
    // // Y.whileTrue(shooterSubsystem.run(shooterSubsystem::aimDown));

    // A.and(B.negate()).whileTrue(m_shooter.run(m_shooter::shooterIn));
    // B.and(A.negate()).whileTrue(m_shooter.run(m_shooter::shooterOut));
    // // X.and(Y.negate()).whileTrue(shooterSubsystem.run(shooterSubsystem::aimUp));
    // // Y.and(X.negate()).whileTrue(shooterSubsystem.run(shooterSubsystem::aimDown));

    // m_shooter.setDefaultCommand(new RunCommand(m_shooter::shooterStop, m_shooter));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonCommand() {
    // autonCommand will run in autonomous
    return drivebase.getAutonomousCommand("New Auto");

  }
}
