package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.usercontrol.GamepadF310;

/**
 * 
 */
public class IntakeCommand extends SequentialCommandGroup {
  /**
   * 
   */
  public IntakeCommand(Intake intake, Indexing indexing
    // , BeamBreak beamBreakBottom
    , GamepadF310 f310 
  ) {
    addCommands(
      intake.runOnce(intake::intakeIn),
      // new WaitUntilCommand(beamBreakBottom::isBeamBroken),
      new WaitUntilCommand(f310::getA),
      indexing.runOnce(indexing::indexIn),
      // new WaitUntilCommand(() -> !beamBreakBottom.isBeamBroken()),
      new WaitUntilCommand(() -> !f310.getA()),
      intake.runOnce(intake::intakeStop),
      new WaitCommand(1.0),
      indexing.runOnce(indexing::indexStop)
    );
    indexing.setLoaded(true);
  }
}
