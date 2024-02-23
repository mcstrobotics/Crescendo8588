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
          intake.runOnce(intake::intakeIn), 
          indexing.runOnce(indexing::indexIn), 
          shooter.runOnce(shooter::shooterOut)
        ),
        // running the intake, index, and shooter for 5 seconds
        new WaitCommand(5.0),
        // stopping everything
        Commands.parallel(
          intake.runOnce(intake::intakeStop),
          indexing.runOnce(indexing::indexStop),
          shooter.runOnce(shooter::shooterStop)
        )
    );
    indexing.setLoaded(false);
  }
}