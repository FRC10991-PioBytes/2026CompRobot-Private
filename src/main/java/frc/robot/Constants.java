// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import java.awt.geom.Rectangle2D;

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
    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    
    public static final double kDriveBaseLength = Units.inchesToMeters(27);
    public static final double kHalfDriveBaseLength = Units.inchesToMeters(13.5);
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
    /*
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps)
        / (kDrivingMotorReduction);
    */
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
    public static final int kAgitatorCanId = 14;

    // Feed forward constants
    public static final double kStaticFF = 0.19906 / 60.0; // Divide by 60 to change from rps to rpm
    public static final double kVelocityFF = 0.126 / 60.0;
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
    public static final double kShooterIdleVelocity = 4200; // RPM
  }

  public static final class LEDConstants {

    public static final int kLEDStripPort = 0;

    public static final Rotation2d kFrontLeftLEDAngularOffset = Rotation2d.fromDegrees(-140);
    public static final Rotation2d kFrontRightLEDAngularOffset = Rotation2d.fromDegrees(-56);
    public static final Rotation2d kBackLeftLEDAngularOffset = Rotation2d.fromDegrees(28);
    public static final Rotation2d kBackRightLEDAngularOffset = Rotation2d.fromDegrees(-47);
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
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
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
    
    // Hub Dimensions
    public static final double kHubSize = Units.inchesToMeters(47.0); 
    public static final double kHubHalfSize = kHubSize / 2.0;

    // Trench/Bump Divider Dimensions
    public static final double kDividerWidth = Units.inchesToMeters(47.0); 
    public static final double kDividerThickness = Units.inchesToMeters(12.0); 
    public static final double kDividerHalfWidth = kDividerWidth / 2.0;
    public static final double kDividerHalfThickness = kDividerThickness / 2.0;

    // Y-Coordinates for Dividers and Centers
    public static final double kLowerDividerY = Units.inchesToMeters(56.34);
    public static final double kUpperDividerY = Units.inchesToMeters(261.35);
    public static final double kMidFieldY = Units.inchesToMeters(158.845);

    // Blue Alliance Objects
    public static final double kBlueCenterX = Units.inchesToMeters(182.11);
    public static final Rectangle2D kBlueHUB = new Rectangle2D.Double(
        kBlueCenterX - kHubHalfSize, 
        kMidFieldY - kHubHalfSize, 
        kHubSize, 
        kHubSize
    );
    
    public static final Rectangle2D kBlueLowerTrenchDivider = new Rectangle2D.Double(
        kBlueCenterX - kDividerHalfWidth, 
        kLowerDividerY - kDividerHalfThickness, 
        kDividerWidth, 
        kDividerThickness
    );
        
    public static final Rectangle2D kBlueUpperTrenchDivider = new Rectangle2D.Double(
        kBlueCenterX - kDividerHalfWidth, 
        kUpperDividerY - kDividerHalfThickness, 
        kDividerWidth, 
        kDividerThickness
    );

    // Red Alliance Objects
    public static final double kRedCenterX = Units.inchesToMeters(469.11);
    public static final Rectangle2D kRedHUB = new Rectangle2D.Double(
        kRedCenterX - kHubHalfSize, 
        kMidFieldY - kHubHalfSize, 
        kHubSize, 
        kHubSize
    );

    public static final Rectangle2D kRedLowerTrenchDivider = new Rectangle2D.Double(
        kRedCenterX - kDividerHalfWidth, 
        kLowerDividerY - kDividerHalfThickness, 
        kDividerWidth, 
        kDividerThickness
    );
        
    public static final Rectangle2D kRedUpperTrenchDivider = new Rectangle2D.Double(
        kRedCenterX - kDividerHalfWidth, 
        kUpperDividerY - kDividerHalfThickness, 
        kDividerWidth, 
        kDividerThickness
    );


    public static final double kFieldHeight = Units.inchesToMeters(317.69);
    public static final double kFieldLength = Units.inchesToMeters(651.52);

    public static final Translation2d kBlueHUBCenter = new Translation2d(4.6256, 4.0345);
    public static final Translation2d kRedHUBCenter = new Translation2d(11.9154, 4.0345);

    private static ArrayList<Pose2d> kBlueScoringPoses = new ArrayList<>();
    private static ArrayList<Pose2d> kRedScoringPoses = new ArrayList<>();

    private static ArrayList<BlueScoringPosition> kBlueScoringPositions = new ArrayList<>();
    private static ArrayList<RedScoringPosition> kRedScoringPositions = new ArrayList<>();

    // Blue Scoring Positions
    public static final BlueScoringPosition kBlueLeftScoringPose1 = new BlueScoringPosition(2.061, 5.173, 4050); //Good
    public static final BlueScoringPosition kBlueLeftScoringPose2 = new BlueScoringPosition(1.76, 6, 4400); //Good
    public static final BlueScoringPosition kBlueLeftScoringPose3 = new BlueScoringPosition(3.083, 6.378, 4050); //Good
    public static final BlueScoringPosition kBlueLeftScoringPose4 = new BlueScoringPosition(1.5, 7.25, 5350); //Good
    public static final BlueScoringPosition kBlueLeftScoringPose5 = new BlueScoringPosition(1.175, 5.93, 4650); //Good
    public static final BlueScoringPosition kBlueLeftScoringPose6 = new BlueScoringPosition(2.67, 6.907, 4400); //Good
    public static final BlueScoringPosition kBlueLeftScoringPose7 = new BlueScoringPosition(1.26, 4.91, 4400); //Good
    
    public static final BlueScoringPosition kBlueCenterScoringPose1 = new BlueScoringPosition(1.82, 4.0345, 4050); //Good
    public static final BlueScoringPosition kBlueCenterScoringPose2 = new BlueScoringPosition(3.04, 4.0345, 3700); //Good

    public static final BlueScoringPosition kBlueRightScoringPose1 = new BlueScoringPosition(2.074, 2.868, 4050); //Good
    public static final BlueScoringPosition kBlueRightScoringPose2 = new BlueScoringPosition(1.776, 2.045, 4400); //Good
    public static final BlueScoringPosition kBlueRightScoringPose3 = new BlueScoringPosition(3.067, 1.701, 4100); //Good
    public static final BlueScoringPosition kBlueRightScoringPose4 = new BlueScoringPosition(1.484, 0.834, 5350); //Good
    public static final BlueScoringPosition kBlueRightScoringPose5 = new BlueScoringPosition(1.19, 2.112, 4650); //Good
    public static final BlueScoringPosition kBlueRightScoringPose6 = new BlueScoringPosition(2.67, 1.162, 4400); //Good

    
    // Red Scoring positions
    public static final RedScoringPosition kRedLeftScoringPose1 = new RedScoringPosition(14.467, 2.868, 4050); //Good
    public static final RedScoringPosition kRedLeftScoringPose2 = new RedScoringPosition(14.765, 2.045, 4400); //Good
    public static final RedScoringPosition kRedLeftScoringPose3 = new RedScoringPosition(13.474, 1.701, 4050); //Good
    public static final RedScoringPosition kRedLeftScoringPose4 = new RedScoringPosition(15.057, 0.834, 5350); //Good
    public static final RedScoringPosition kRedLeftScoringPose5 = new RedScoringPosition(15.351, 2.112, 4650);//Good
    public static final RedScoringPosition kRedLeftScoringPose6 = new RedScoringPosition(13.871, 1.162, 4400); //Good
    public static final RedScoringPosition kRedLeftScoringPose7 = new RedScoringPosition(15.28, 3.124, 4400); //Good
    
    public static final RedScoringPosition kRedCenterScoringPose1 = new RedScoringPosition(14.721, 4.0345, 4050); //Good
    public static final RedScoringPosition kRedCenterScoringPose2 = new RedScoringPosition(14, 4.0345, 3700); //Good

    public static final RedScoringPosition kRedRightScoringPose1 = new RedScoringPosition(14.48, 5.173, 4050); //Good
    public static final RedScoringPosition kRedRightScoringPose2 = new RedScoringPosition(14.781, 6, 4400); //Good
    public static final RedScoringPosition kRedRightScoringPose3 = new RedScoringPosition(13.458, 6.378, 4050); //Good
    public static final RedScoringPosition kRedRightScoringPose4 = new RedScoringPosition(15.041, 7.25, 5350); //Good
    public static final RedScoringPosition kRedRightScoringPose5 = new RedScoringPosition(15.366, 5.93, 4650); //Good
    public static final RedScoringPosition kRedRightScoringPose6 = new RedScoringPosition(13.871, 6.907, 4400); //Good

    public static ArrayList<Pose2d> getBlueScoringPoses() {
      return kBlueScoringPoses;
    }
    
    public static ArrayList<Pose2d> getRedScoringPoses() {
      return kRedScoringPoses;
    }

    public static ArrayList<BlueScoringPosition> getBlueScoringPositions() {
      return kBlueScoringPositions;
    }

    public static ArrayList<RedScoringPosition> getRedScoringPositions() {
      return kRedScoringPositions;
    }
    
    /** Represents a Blue 2D pose pointed at the Blue HUB with an RPM */
    public static class BlueScoringPosition {
      private Pose2d kScoringPose;
      private double kShooterRPM;
      
      public BlueScoringPosition(double x, double y, double RPM)
      {
        Translation2d scoringTranslation = new Translation2d(x, y);
        kScoringPose = new Pose2d(scoringTranslation, kBlueHUBCenter.minus(scoringTranslation).getAngle());
        kBlueScoringPoses.add(kScoringPose);

        kShooterRPM = RPM;

        kBlueScoringPositions.add(this);
      }

      public static double getNearestRPM(Pose2d robotPose)
      {
        Pose2d closestPose = robotPose.nearest(kBlueScoringPoses);

        for (int i = 0; i < kBlueScoringPoses.size(); i++)
        {
          if (kBlueScoringPoses.get(i).equals(closestPose))
          {
            return kBlueScoringPositions.get(i).getRPM();
          }
        }

        return 0.0;
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

    /** Represents a Red 2D pose pointed at the Red HUB with an RPM */
    public static class RedScoringPosition {
      private Pose2d kScoringPose;
      private double kShooterRPM;
      
      public RedScoringPosition(double x, double y, double RPM)
      {
        Translation2d scoringTranslation = new Translation2d(x, y);
        kScoringPose = new Pose2d(scoringTranslation, kRedHUBCenter.minus(scoringTranslation).getAngle());
        kRedScoringPoses.add(kScoringPose);

        kShooterRPM = RPM;

        kRedScoringPositions.add(this);
      }

      public static double getNearestRPM(Pose2d robotPose)
      {
        Pose2d closestPose = robotPose.nearest(kRedScoringPoses);

        for (int i = 0; i < kRedScoringPoses.size(); i++)
        {
          if (kRedScoringPoses.get(i).equals(closestPose))
          {
            return kRedScoringPositions.get(i).getRPM();
          }
        }

        return 0.0;
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
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final String kLimelightName = "limelight";
  }
}