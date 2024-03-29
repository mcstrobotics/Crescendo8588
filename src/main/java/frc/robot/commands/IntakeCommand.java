package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.BeamBreak;
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
    , BeamBreak beamBreakBottom
    // , GamepadF310 f310 
  ) {
    addCommands(
      Commands.parallel(
        intake.runOnce(intake::intake),
        new WaitUntilCommand(beamBreakBottom::isBeamBroken)
        // ,new WaitUntilCommand(f310::getA)
      ),
      Commands.parallel(
        indexing.runOnce(indexing::intake),
        new WaitUntilCommand(() -> !beamBreakBottom.isBeamBroken())
        // ,new WaitUntilCommand(() -> !f310.getA())
      ),
      Commands.parallel(
        intake.runOnce(intake::stop),
        new WaitCommand(1.0)
      ),
      indexing.runOnce(indexing::stop)
    );
    indexing.setLoaded(true);
  }
}
