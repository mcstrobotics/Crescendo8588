// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // TODO !!! WILL NEED TO CHANGE TRACK WIDTH AND WHEEL BASE
    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    // THIS MAY NEED TO BE CHANGED
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
      / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }
  
  public static final class MotorContants {
    public static final double kArmSpeed      = 0.5; // meters per second
    public static final double kIntakeSpeed   = 1.0; // meters per second
    public static final double kShootingSpeed = 3.0; // meters per second

    public static final int kMotorCurrentLimit = 50; // amps
  }

  /** Constants for the Intake Subsystem */
  public static final class IntakeConstants {
    // CAN IDs
    public static final int kIntakeCanId = 20;

    // MEASUREMENTS !!!!!!!!! NOT FINAL
    public static final double kWheelDiameter = 4.0 / 12.0; // meters

    // UNIT CONVERSION
    public static final double kEncoderPositionFactor = kWheelDiameter * Math.PI; // meters
    public static final double kEncoderVelocityFactor = (kWheelDiameter * Math.PI) / 60.0; // meters per second

    // PID tuning
    public static final double kP  = 1;
    public static final double kI  = 0;
    public static final double kD  = 0;
    public static final double kFF = 0;
  }

  /** Constants for the Indexing Subsystem */
  public static final class IndexingConstants {
    // CAN IDs
    public static final int kLeftCanId = 21;
    public static final int kRightCanId = 22;

    // MEASUREMENTS !!!!!!!!! NOT FINAL
    public static final double kWheelDiameter = 3.0 / 12.0; // meters

    // UNIT CONVERSION
    public static final double kLeftEncoderPositionFactor = kWheelDiameter * Math.PI; // meters
    public static final double kLeftEncoderVelocityFactor = (kWheelDiameter * Math.PI) / 60.0; // meters per second

    public static final double kRightEncoderPositionFactor = kLeftEncoderPositionFactor; // meters
    public static final double kRightEncoderVelocityFactor = kLeftEncoderVelocityFactor; // meters per second

    // PID tuning
    public static final double kP  = 1;
    public static final double kI  = 0;
    public static final double kD  = 0;
    public static final double kFF = 0;
  }

  /** Constants for the Shooter Subsystem */
  public static final class ShooterConstants {
    // CAN IDs
    public static final int kAimingCanId = 23;

    public static final int kBottomCanId = 24;
    public static final int kTopCanId = 25;

    // MEASUREMENTS
    public static final double kBottomWheelDiameter = 3.0 / 12.0; // meters
    public static final double kTopWheelDiameter    = 4.0 / 12.0; // meters

    // UNIT CONVERSION
    public static final double kBottomEncoderPositionFactor = kBottomWheelDiameter * Math.PI; // meters
    public static final double kBottomEncoderVelocityFactor = (kBottomWheelDiameter * Math.PI) / 60.0; // meters per second

    public static final double kTopEncoderPositionFactor = kTopWheelDiameter * Math.PI; // meters
    public static final double kTopEncoderVelocityFactor = (kTopWheelDiameter * Math.PI) / 60.0; // meters per second

    // PID tuning
    public static final double kBottomP  = 1;
    public static final double kBottomI  = 0;
    public static final double kBottomD  = 0;
    public static final double kBottomFF = 0;

    public static final double kTopP  = 1;
    public static final double kTopI  = 0;
    public static final double kTopD  = 0;
    public static final double kTopFF = 0;
  }

  /** Constants for the Wrist Constants */
  public static final class WristConstants {
    // CAN IDs
    public static final int kArmCanId = 23;

    // THROUGHBORE ENCODER
    public static final int kCountsPerRev = 8192;

    // UNIT CONVERSION
    public static final double kEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
    
    // // PID tuning
    // public static final double kP = 1;
    // public static final double kI = 0;
    // public static final double kD = 0;
    // public static final double kFF = 0;

    // These are fake gains; in actuality these must be determined individually for each robot
    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;
    
    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kWristOffsetRads = 0.5;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
      kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}