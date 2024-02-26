package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.usercontrol.GamepadF310;

/**
 * 
 */
public class IntakeNoteCommand extends SequentialCommandGroup {
  /**
   * 
   */
  public IntakeNoteCommand(Intake intake, Indexing indexing
    // , BeamBreak beamBreakBottom
    , CommandXboxController driverXbox 
  ) {
    addCommands(
      intake.runOnce(intake::intakeIn),
      // new WaitUntilCommand(beamBreakBottom::isBeamBroken),
      // Making a new waitUntilCommand that runs until the driverXbox's A button is pressed
      new WaitUntilCommand(driverXbox.a()),
      indexing.runOnce(indexing::indexIn),
      // new WaitUntilCommand(() -> !beamBreakBottom.isBeamBroken()),
      new WaitUntilCommand(driverXbox.a()),
      intake.runOnce(intake::intakeStop),
      new WaitCommand(1.0),
      indexing.runOnce(indexing::indexStop)
    );
  }
}
