package frc.robot.commands;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.BeamBreak;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
// import frc.robot.usercontrol.GamepadF310;
import frc.utils.LookupTable;

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
  public ShootCommand(Intake intake, Indexing indexing, Shooter shooter, Vision vision, BeamBreak beamBreakTop, // this has way too many parameters but idk what to do about it
      DoubleLogEntry shotDistance, DoubleLogEntry shotAngle, 
      LookupTable speakerLookupTable, LookupTable ampLookupTable
  ) {
    this.shotDistance = shotDistance;
    this.shotAngle = shotAngle;
    this.vision = vision;

    int tagId = vision.getAprilTagId(vision.camera.getLatestResult());
    System.out.println(tagId);

    // TODO WE WILL HAVE A CASE STATEMENT HERE TO DO DIFFERENT STUFF BASED ON THE IDS
    // switch (tagId) {
    //   case value:
    //     break;
    //   case value:
    //     break;
    //   case value:
    //     break;
    //   case value:
    //     break;
    //   case value:
    //     break;
    //   case value:
    //     break;

    //   default:
    //     break;
    // }

    // TODO TARGETHEIGHT AND THE LOOKUPTABLE ARE IN THE CASE STATEMENT, THE BELOW LINES OF CODE ARE TEMPORARY
    double targetHeight = 1; // meters
    LookupTable lookupTable = speakerLookupTable;

    double distance = vision.getDistanceToTarget(targetHeight); // meters
    double angle = lookupTable.getAngleFromDistance(distance); // radians

    addCommands(
        Commands.parallel(
            // taking all the notes out
            shooter.runOnce(shooter::shoot),
            indexing.runOnce(indexing::shoot)),
        new WaitUntilCommand(beamBreakTop::isBeamBroken),
        new WaitCommand(2.0),
        Commands.parallel(
            // stopping everything
            shooter.runOnce(shooter::stop),
            indexing.runOnce(indexing::stop)),
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