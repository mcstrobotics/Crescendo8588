package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
/**
 *
 */
public class PurgeCommand extends SequentialCommandGroup {
  /**
   *
   */
  public PurgeCommand(Intake intake, Indexing indexing, Shooter shooter) {
    // adding commands
    addCommands (
        // expelling all objects from the robot
        Commands.parallel(
          intake.runOnce(intake::intake), 
          indexing.runOnce(indexing::intake), 
          shooter.runOnce(shooter::intake)
        ),
        // running the intake, index, and shooter for 5 seconds
        new WaitCommand(5.0),
        // stopping everything
        Commands.parallel(
          intake.runOnce(intake::stop),
          indexing.runOnce(indexing::stop),
          shooter.runOnce(shooter::stop)
        )
    );
    indexing.setLoaded(false);
  }
}