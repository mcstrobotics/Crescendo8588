package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface DriveSubsystemInterface extends Subsystem {
    void periodic();

    Pose2d getPose();

    void resetOdometry(Pose2d pose);

    void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit);

    void setX();

    void setModuleStates(SwerveModuleState[] desiredStates);

    void resetEncoders();

    void zeroHeading();

    double getHeading();

    double getTurnRate();
}
