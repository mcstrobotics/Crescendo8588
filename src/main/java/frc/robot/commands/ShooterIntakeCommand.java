package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.BeamBreaker;
import frc.robot.usercontrol.GamepadF310;
/**
 *
 */
public class ShooterIntakeCommand extends SequentialCommandGroup {
  /**
   *
   */
  public ShooterIntakeCommand (Indexing indexing, Shooter shooter
    // , BeamBreak beamBreakTop
    , GamepadF310 f310
  ) {
    addCommands(
      // taking everything in
      Commands.parallel(
        // taking the note in from shooter
        shooter.runOnce(shooter::shooterIntake),
        indexing.runOnce(indexing::shooterIntake)
      ),
      // new WaitUntilCommand(beamBreakTop::isBeamBroken),
      // new WaitUntilCommand(() -> !beamBreakTop.isBeamBroken()),
      new WaitCommand(1),
      Commands.parallel(
        // stopping everything
        shooter.runOnce(shooter::stop),
        indexing.runOnce(indexing::stop)
      )
    );
    indexing.setLoaded(true);
  }
}
