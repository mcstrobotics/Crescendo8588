package frc.robot.commands;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.BeamBreak;

import frc.robot.subsystems.Vision;
// import frc.robot.usercontrol.GamepadF310;

/**
 *
 */
public class ShootCommand extends SequentialCommandGroup {
  DoubleLogEntry shotDistance;
  DoubleLogEntry shotAngle;

  Vision vision;

  /**
   *
   */
  public ShootCommand(Intake intake, Indexing indexing, Shooter shooter, BeamBreak beamBreakTop,
      DoubleLogEntry shotDistance, DoubleLogEntry shotAngle
  // , GamepadF310 f310
  ) {
    this.shotDistance = shotDistance;
    this.shotAngle = shotAngle;

    double distance = vision.getDistanceToTarget(1.0); // meters
    double angle = 0; // radians

    addCommands(
        Commands.parallel(
            // taking all the notes out
            shooter.runOnce(shooter::shoot),
            indexing.runOnce(indexing::shoot)),
        // waiting until the top beam is broken then waiting 4 seconds to turn
        // everything off
        new WaitUntilCommand(beamBreakTop::isBeamBroken),
        new WaitCommand(4.0),
        // GET DISTANCE AND STORE IT IN A VARIABLE HERE
        Commands.parallel(
            // stopping everything
            shooter.runOnce(shooter::stop),
            indexing.runOnce(indexing::stop)),
        // LOG IT HERE
        Commands.runOnce(() -> {
          logShot(distance, angle);
        }));
    indexing.setLoaded(false);
  }

  // we log the shots cus we need for the lookuptable
  public void logShot(double distance, double angle) {
    shotDistance.append(distance);
    shotAngle.append(angle);
  }
}