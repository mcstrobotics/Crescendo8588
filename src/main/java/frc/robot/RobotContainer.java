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

// SUBSYSTEMS
import frc.robot.subsystems.intake.IntakeChassis;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  private GamepadF310 f310 = new GamepadF310(0);

  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem(new IntakeChassis(
      new CANSparkMax(IntakeConstants.kIntakeCanId, CANSparkLowLevel.MotorType.kBrushless)));

  private AutonCommand autonCommand = new AutonCommand(m_robotDrive);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(f310.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(f310.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(f310.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Mihir - Guys, this is what I was talking about when we were talking about command based programming:
    // Intake Bindings
    new Trigger(f310::getA) // bindings are TBD
        .onTrue(Commands.runOnce( // fires once when the button is pressed
            intakeSubsystem::intakeIn)) // invoke intake in method
        .onFalse(Commands.runOnce( // fires once when the button is released
            intakeSubsystem::intakeStop)); // invoke stop intake method
    new Trigger(f310::getB)
        .onTrue(Commands.runOnce(
            intakeSubsystem::intakeOut))
        .onFalse(Commands.runOnce(
            intakeSubsystem::intakeStop));
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
