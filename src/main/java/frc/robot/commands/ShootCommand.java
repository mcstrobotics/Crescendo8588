package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.BeamBreaker;
import frc.robot.usercontrol.GamepadF310;
/**
 *
 */
public class ShootCommand extends SequentialCommandGroup {
  /**
   *
   */
  public ShootCommand (Intake intake, Indexing indexing, Shooter shooter
//   , BeamBreaker beamBreakTop
    , GamepadF310 f310
  ) {
    addCommands(
      Commands.parallel(
        // taking all the notes out
        shooter.runOnce(shooter::shoot),
        indexing.runOnce(indexing::shoot)
      ),
      // waiting until the top beam is broken then waiting 4 seconds to turn everything off
    //   new WaitUntilCommand(beamBreakTop::isBeamBroken),
      new WaitCommand(4.0),
      Commands.parallel(
        // stopping everything
        shooter.runOnce(shooter::stop),
        indexing.runOnce(indexing::stop)
      )
    );
  }
}