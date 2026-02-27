// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI);

  private final Field2d field = new Field2d();

  /*
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      //Rotation2d.kZero,
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });
  */

  SwerveDrivePoseEstimator m_poseEstimator;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    
    // Set up a callback that PathPlanner calls every 20ms
    PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
        // Only do this in simulation so you don't break the real robot!
        if (RobotBase.isSimulation()) {
            // Forcibly update your odometry to exactly match the target
            this.resetPose(targetPose); 
        }
    });

    RobotConfig config;
    
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
      this::getPose, 
      this::resetPose,
      this::getRobotRelativeSpeeds,
      (speeds, feedforwards) -> drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, 1, false),
      new PPHolonomicDriveController(
        new PIDConstants(0, 0, 0), 
        new PIDConstants(0, 0, 0)),
      config,
      () -> false,
      this);
    }
    catch (Exception e) {
      e.printStackTrace();
    }

    zeroHeading();


    m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics, 
      Rotation2d.fromDegrees(this.getHeading()),
      new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }, 
        new Pose2d());
    
    //resetOdometryWithAprilTags();

  }

  @Override
  public void periodic() {
    /*
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        //Rotation2d.kZero,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    */

    updateOdometry();

    SmartDashboard.putNumber("Drive/Gyro Angle: ", this.getHeading());
    SmartDashboard.putNumber("Drive/Gyro reading", m_gyro.getRotation2d().getDegrees());

    //LimelightHelpers.SetRobotOrientation("limelight", this.getHeading(), 0, 0, 0, 0, 0);
    //LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    //this.getField().getObject("mt2 pose").setPose(mt2.pose);

    
    
    

    Pose2d robotPose = getPose();
    field.setRobotPose(robotPose);
    SmartDashboard.putData("Game Info/Field", field);

    SmartDashboard.putData("Drive/Swerve Drive", this);

    //field.getObject("Blue scoring poses").setPoses(FieldConstants.BlueScoringPosition.getBlueScoringPoses());
    //field.getObject("Red scoring poses").setPoses(FieldConstants.RedScoringPosition.getRedScoringPoses());
    
  }


  public void updateOdometry() {
    m_poseEstimator.update(
        Rotation2d.fromDegrees(this.getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    boolean doRejectUpdate = false;
    
    LimelightHelpers.SetRobotOrientation("limelight", this.getHeading(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    //this.getField().getObject("mt2 pose").setPose(mt2.pose);
    if (mt2 != null) {
      if (Math.abs(this.getTurnRate()) > 720 || mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate)
      {
        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 999999));
        m_poseEstimator.addVisionMeasurement(
          mt2.pose, 
          mt2.timestampSeconds);
      }
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    
    //return m_odometry.getPoseMeters();
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    /*
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        //Rotation2d.kZero,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
    */
    
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(this.getHeading()),
        //Rotation2d.kZero,
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

    //m_poseEstimator.resetRotation(pose.getRotation());
  }

  public void resetOdometryWithAprilTags() {
    try {
      LimelightHelpers.SetIMUMode("limelight", 4);
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      if (mt1.tagCount > 0)
      {
        resetPose(mt1.pose);
        System.out.println("Set pose to " + mt1.pose.getX() + " x and " + mt1.pose.getY() + "y and " + mt1.pose.getRotation().getDegrees() + " degrees");
      }
    }
    catch (Exception e) {
      System.out.println("Couldn't get pose");
      e.printStackTrace();
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, double speedMult, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * speedMult;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * speedMult;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed * speedMult;
    
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeedDelivered, -ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(this.getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

      SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }
    else {
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(this.getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

      SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }
    

    
  }

  public void stop()
  {
    drive(0,0,0,0,true);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] swerveModuleStates = {m_frontLeft.getState(), m_frontRight.getState(), m_rearLeft.getState(), m_rearRight.getState()};
    return swerveModuleStates;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public Field2d getField() {
    return field;
  }
  
  
  //Zeroes the heading of the robot.
  public void zeroHeading() {
    m_gyro.reset();
  }
  

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
  */
  
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
  


  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
  */
  
  public double getTurnRate() {
    //return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return m_gyro.getRate();
  }
  
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getAngle().getRadians(), null);
    builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getVelocity(), null);

    builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getAngle().getRadians(), null);
    builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getVelocity(), null);

    builder.addDoubleProperty("Back Left Angle", () -> m_rearLeft.getAngle().getRadians(), null);
    builder.addDoubleProperty("Back Left Velocity", () -> m_rearLeft.getVelocity(), null);

    builder.addDoubleProperty("Back Right Angle", () -> m_rearRight.getAngle().getRadians(), null);
    builder.addDoubleProperty("Back Right Velocity", () -> m_rearRight.getVelocity(), null);

    builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getRadians(), null);
  }
}
