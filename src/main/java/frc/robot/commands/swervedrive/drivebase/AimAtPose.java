// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import static frc.robot.Constants.Vision.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

import frc.robot.subsystems.Vision;

/**
 * An example command that uses an example subsystem.
 */
public class AimAtPose extends Command {
  private final SwerveSubsystem swerve;
  // private final DoubleSupplier vX, vY;
  // private final DoubleSupplier heading;

  private final PhotonCamera camera;

  // Constants such as camera and target height stored. Change per robot and goal!
  final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
  final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
  // Angle between horizontal and the camera.
  final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(0);
  // How far from the target we want to be
  final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  // PID constants should be tuned per robot
  final double LINEAR_P = 0.1;
  final double LINEAR_D = 0.0;
  PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0.0;
  PIDController headingController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  Pose2d targetPose;

  private Vision vision;

  /**
   * 
   */
  public AimAtPose(
      SwerveSubsystem swerve,
      // DoubleSupplier vX, DoubleSupplier vY,
      Vision vision,
      Pose2d targetPose) {
    this.swerve = swerve;
    // this.vX = vX;
    // this.vY = vY;
    this.vision = vision;
    this.camera = vision.camera;
    this.targetPose = targetPose;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed;
    double rotationSpeed;

    Translation2d translation;
    ChassisSpeeds desiredSpeeds;
    Pose2d robotPose;

    var visionEstimatedPose = vision.getEstimatedGlobalPose();
    if (visionEstimatedPose.isPresent()) 
      robotPose = visionEstimatedPose.get().estimatedPose.toPose2d();
    else robotPose = swerve.getPose();

    double range = PhotonUtils.getDistanceToPose(robotPose, targetPose);

    // Use this range as the measurement we give to the PID controller.
    // -1.0 required to ensure positive PID controller effort _increases_ range
    forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);

    // Also calculate angular power
    // -1.0 required to ensure positive PID controller effort _increases_ yaw
    rotationSpeed = -headingController.calculate(PhotonUtils.getYawToPose(robotPose, targetPose).getRadians(), 0);

    desiredSpeeds = swerve.getTargetSpeeds(0, 0, new Rotation2d(rotationSpeed * Math.PI));

    // Limit velocity to prevent tippy
    translation = SwerveController.getTranslation2d(desiredSpeeds);
    translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
        Constants.LOOP_TIME, Constants.ROBOT_MASS, List.of(Constants.CHASSIS),
        swerve.getSwerveDriveConfiguration());
    SmartDashboard.putNumber("LimitedTranslation", translation.getX());
    SmartDashboard.putString("Translation", translation.toString());

    swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}