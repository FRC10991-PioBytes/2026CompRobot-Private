// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Collection;


import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    public static final double kFastSpeedMultiplier = 0.75;
    public static final double kSlowSpeedMultiplier = 0.4;
    

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
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
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 6;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / (kDrivingMotorReduction);
  }

  public static final class IntakeConstants {
    public static final int kLeaderPivotCanId = 12;
    public static final int kLeaderRollerCanId = 13;

    public static final double kIntakeInSpeed = 1;
    public static final double kIntakeOutSpeed = -1;

    public static final double kIntakePivotUpSpeed = 0.3;
    public static final double kIntakePivotDownSpeed = -0.1;
    public static final double kIntakePivotHoldSpeed = 0.1;

    // In radians
    public static final Rotation2d kIntakeRetractedEncoderPosition = Rotation2d.kZero;
    public static final Rotation2d kIntakeExtendedEncoderPosition = new Rotation2d(0.1);

    public static final double kIntakeGravityCosVoltage = 0.1;
    public static final double kStaticFF = 0;
    public static final double kVelocityFF = 0;
    public static final double kAccelerationFF = 0;

  }

  public static final class FeederConstants {
    // CAN ID
    public static final int kLeftFeederCanId = 9;

    // Feed forward constants
    public static final double kStaticFF = 0.19906 / 60.0; // Divide by 60 to change from rps to rpm
    public static final double kVelocityFF = 0.12557 / 60.0;
    public static final double kAccelerationFF = 0.015522 / 60.0;

    // PID constants
    public static final double kP = 2.9426E-05;
    public static final double kD = 0;

    public static final int kFeederMaxAcceleration = 2000; // RPM per second
  }

  public static final class ShooterConstants {
    // CAN IDs
    public static final int kLeaderShooterCanId = 10;
    public static final int kRightShooterCanId = 11;

    // Feed forward constants
    public static final double kStaticFF = 0.20042 / 60.0; // Divide by 60 to change from rps to rpm
    public static final double kVelocityFF = 0.12391 / 60.0;
    public static final double kAccelerationFF = 0.011037 / 60.0;

    // PID constants
    public static final double kP = 1.0486E-05;
    public static final double kD = 0;

    public static final int kShooterMaxAcceleration = 1500; // RPM per second
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;

    public static final int kManipulatorControllerPort = 1;
    public static final int leftStickY = 1;
    public static final int leftStickX = 0;
    public static final int rightStickY = 5;
    public static final int rightStickX = 4;
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;
    public static final int buttonA = 1;
    public static final int buttonB = 2;
    public static final int buttonX = 3;
    public static final int buttonY = 4;
    public static final int bumperLeft = 5;
    public static final int bumperRight = 6;
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

  // All values are field centric from the blue origin in meters
  public static final class FieldConstants {
    
    public static final Translation2d kBlueHUB = new Translation2d(4.625594, 4.034536);
    public static final Translation2d kRedHUB = new Translation2d(11.915394, 4.034536);

    
    public static final BlueScoringPosition kBlueScorePose1 = new BlueScoringPosition(2.577, 5.98, 4000);
    public static final BlueScoringPosition kBlueScorePose2 = new BlueScoringPosition(11.915394, 4.034536, 4000);


    /** Represents a 2D pose pointed at the HUB with an RPM */
    public static class BlueScoringPosition {
      private Pose2d kScoringPose;
      private double kShooterRPM;

      private static Collection<Pose2d> kBlueScoringPoses = new ArrayList<>();
      
      public BlueScoringPosition(double x, double y, double RPM)
      {
        Translation2d scoringTranslation = new Translation2d(x, y);
        kScoringPose = new Pose2d(scoringTranslation, new Rotation2d(kBlueHUB.minus(scoringTranslation).getAngle().getRadians()));
        kBlueScoringPoses.add(kScoringPose);

        kShooterRPM = RPM;
      }
      
      public static Collection<Pose2d> getBlueScoringPoses()
      {
        return kBlueScoringPoses;
      }

      public Pose2d getPose2d()
      {
        return kScoringPose;
      }

      public Translation2d getTranslation2d()
      {
        return kScoringPose.getTranslation();
      }

      public Rotation2d getRotation2d()
      {
        return kScoringPose.getRotation();
      }

      public double getRPM()
      {
        return kShooterRPM;
      }
      
    }


    public static final Pose2d getBlueScorePose(double x, double y)
    {
      Translation2d scoringTranslation = new Translation2d(x, y);
      return new Pose2d(scoringTranslation, new Rotation2d(kBlueHUB.minus(scoringTranslation).getAngle().getRadians()));
    }

    public static final Pose2d getRedScorePose(double x, double y)
    {
      Translation2d scoringTranslation = new Translation2d(x, y);
      return new Pose2d(scoringTranslation, new Rotation2d(kRedHUB.minus(scoringTranslation).getAngle().getRadians()));
    }
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final String kLimelightName = "limelight";
  }
}