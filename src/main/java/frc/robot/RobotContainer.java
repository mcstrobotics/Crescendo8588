// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.PurgeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterIntakeCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.commands.swervedrive.drivebase.VisionTest;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Indexing;
// SUBSYSTEMS
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
//import frc.robot.usercontrol.GamepadF310;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.utils.LookupTable;

// import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // OLD Swerve stuff
  // private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  // private AutonCommand autonCommand = new AutonCommand(m_robotDrive);
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "/swerve/maxswerve/"));

  // replaced with CommandXboxController
  // private GamepadF310 f310 = new GamepadF310(0);
  final CommandXboxController driverXbox = new CommandXboxController(1);

  // making subsystem objects
  private final Intake m_intake = new Intake();
  private final Indexing m_indexing = new Indexing();
  private final Shooter m_shooter = new Shooter();
  private final Vision m_vision = new Vision(drivebase);

  private final BeamBreak m_beamBottom = new BeamBreak(0);
  private final BeamBreak m_beamTop = new BeamBreak(1);

  DoubleLogEntry shotDistance; // meters
  DoubleLogEntry shotAngle; // radians

  // hardcoded lookup tables
  private LookupTable speakerLookupTable = new LookupTable(
    new double[] {1, 2, 3, 4, 5}, // radians
    new double[] {1, 2, 3, 4, 5}  // meters
  );
  private LookupTable ampLookupTable = new LookupTable(
    new double[] {1, 2, 3, 4, 5}, // radians
    new double[] {1, 2, 3, 4, 5}  // meters
  );

//   private final IntakeCommand m_intakeCommand = new IntakeCommand(m_intake, m_indexing, m_beamBottom);
//   private final PurgeCommand m_purgeCommand = new PurgeCommand(m_intake, m_indexing, m_shooter);
//   private final ShooterIntakeCommand m_shooterIntakeCommand = new ShooterIntakeCommand(m_indexing, m_shooter, m_beamTop);
//   private final ShootCommand m_shootCommand = new ShootCommand(
//     m_intake, m_indexing, m_shooter, m_vision, 
//     m_beamTop, 
//     shotDistance, shotAngle, 
//     speakerLookupTable, ampLookupTable
//     );

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Starts recording to data log
    DataLogManager.start();

    // Set up custom log entries
    DataLog log = DataLogManager.getLog();
    shotDistance = new DoubleLogEntry(log, "/shots/distance");
    shotAngle = new DoubleLogEntry(log, "/shots/angle");

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

    VisionTest m_turnToTargetCommand = new VisionTest(drivebase,
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
            OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
            OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
            OperatorConstants.RIGHT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getRightY(),
            OperatorConstants.RIGHT_Y_DEADBAND),
        m_vision.camera);

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
        !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle
            // closedAbsoluteDriveAdv
            : driveFieldOrientedDirectAngleSim
        );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxControllerXbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    driverXbox.y().whileTrue(drivebase.sysIdDriveMotorCommand());

    // new Trigger(() -> !m_indexing.isLoaded()).and(driverXbox.a()).onTrue(m_intakeCommand);
    // new Trigger(m_indexing::isLoaded).and(driverXbox.a()).onTrue(m_shootCommand);
    // new Trigger(driverXbox.b()).onTrue(m_purgeCommand);
    // new Trigger(() -> !m_indexing.isLoaded()).and(driverXbox.x()).onTrue(m_shooterIntakeCommand);
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
