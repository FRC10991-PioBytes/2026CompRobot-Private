// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.LimelightHelpers.PoseEstimate;
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

  SwerveDrivePoseEstimator m_poseEstimator;

  private LimelightHelpers.PoseEstimate mt2PoseEstimate = null;
  private ArrayList<Pose2d> llReadings = new ArrayList<Pose2d>();
  private boolean isAddingToList = false;

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
      (speeds, feedforwards) -> driveRobotRelative(speeds),
      new PPHolonomicDriveController(
        new PIDConstants(4, 0, 0), 
        new PIDConstants(4, 0, 0)),
      config,
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
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

  }

  @Override
  public void periodic() {

    updateOdometry();
    SmartDashboard.putNumber("Wheel angle", m_frontLeft.getState().angle.getRotations());
    SmartDashboard.putNumber("Drive/Gyro Angle: ", this.getHeading());

    Pose2d robotPose = getPose();
    field.setRobotPose(robotPose);
    SmartDashboard.putData("Game Info/Field", field);

    SmartDashboard.putData("Drive/Swerve Drive", this);
    SmartDashboard.putNumber("pose estimator yaw", getPose().getRotation().getDegrees());
    //field.getObject("Blue scoring poses").setPoses(FieldConstants.getBlueScoringPoses());
    //field.getObject("Red scoring poses").setPoses(FieldConstants.getRedScoringPoses());
    
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

    LimelightHelpers.SetRobotOrientation("limelight", this.getPose().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    
    try {
      mt2PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

      if (isGoodPoseEstimate(mt2PoseEstimate)) {
        if (isAddingToList) {
          llReadings.add(mt2PoseEstimate.pose);
        }

        if (mt2PoseEstimate.tagCount > 1) {
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 999999));
          m_poseEstimator.addVisionMeasurement(
            mt2PoseEstimate.pose, 
            mt2PoseEstimate.timestampSeconds);
        }
        else if (mt2PoseEstimate.avgTagDist > 3) {
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
            0.24,
            0.24,
            999999));
          m_poseEstimator.addVisionMeasurement(
            mt2PoseEstimate.pose, 
            mt2PoseEstimate.timestampSeconds);
        }
        else if (mt2PoseEstimate.avgTagDist > 2) {
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
            0.14,
            0.14,
            999999));
          m_poseEstimator.addVisionMeasurement(
            mt2PoseEstimate.pose, 
            mt2PoseEstimate.timestampSeconds);
        }
        else if (mt2PoseEstimate.avgTagDist > 1) {
          m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(
            0.07,
            0.07,
            999999));
          m_poseEstimator.addVisionMeasurement(
            mt2PoseEstimate.pose, 
            mt2PoseEstimate.timestampSeconds);
        }
      }
    }
    catch (Exception e) {
    }

    /*
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Blue) {
      LimelightHelpers.SetRobotOrientation("limelight", this.getHeading(), 0, 0, 0, 0, 0);
      }
      else if (DriverStation.getAlliance().get() == Alliance.Red) {
        LimelightHelpers.SetRobotOrientation("limelight", this.getHeading() + 180, 0, 0, 0, 0, 0);
      }
    }
    */

    //LimelightHelpers.SetRobotOrientation("limelight", getHeading(), 0, 0, 0, 0, 0);
    /*
    try {
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      field.getObject("mt2 pose").setPose(mt2.pose);
      if (mt2 != null) {
        if (Math.abs(this.getTurnRate()) < 720 && mt2.tagCount > 0)
        {
          if (mt2.pose.getX() > (halfDriveBase)  && mt2.pose.getY() > (halfDriveBase)) {
            if (mt2.pose.getX() < (16.541 - halfDriveBase) && mt2.pose.getY() < (8.069 - halfDriveBase)) {
              m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 999999));
              m_poseEstimator.addVisionMeasurement(
                mt2.pose, 
                mt2.timestampSeconds);
            }
          }
          
        }
      }
    }
    catch (Exception e) {
      
    }
    */
  }


  public boolean isGoodPoseEstimate(LimelightHelpers.PoseEstimate poseEstimate) {
    // If reading is clearly incorrect
    if (poseEstimate == null || poseEstimate.tagCount < 1) {
      return false;
    }

    // If spinning too quickly
    if (Math.abs(this.getTurnRate()) > 360) {
      return false;
    }

    // If more than four meters away
    if (poseEstimate.avgTagDist > 4) {
      return false;
    }

    Pose2d pose = poseEstimate.pose;

    // If outside of the field pose.getX() > (FieldConstants.kfieldLength - DriveConstants.kHalfDriveBaseLength
    if (pose.getX() < (DriveConstants.kHalfDriveBaseLength)  || pose.getX() > (FieldConstants.kFieldLength - DriveConstants.kHalfDriveBaseLength)) {
      return false;
    }
    else if (pose.getY() < (DriveConstants.kHalfDriveBaseLength) || pose.getY() > (FieldConstants.kFieldHeight - DriveConstants.kHalfDriveBaseLength)) {
      return false;
    }

    // If inside blue field components
    if (FieldConstants.kBlueUpperTrenchDivider.contains(pose.getX(), pose.getY())) {
      return false;
    }
    if (FieldConstants.kBlueHUB.contains(pose.getX(), pose.getY())) {
      return false;
    }
    if (FieldConstants.kBlueLowerTrenchDivider.contains(pose.getX(), pose.getY())) {
      return false;
    }
    // If inside red field components
    if (FieldConstants.kRedUpperTrenchDivider.contains(pose.getX(), pose.getY())) {
      return false;
    }
    if (FieldConstants.kRedHUB.contains(pose.getX(), pose.getY())) {
      return false;
    }
    if (FieldConstants.kRedLowerTrenchDivider.contains(pose.getX(), pose.getY())) {
      return false;
    }

    return true;

    /*
    // If X is inside blue field components
    if (pose.getX() > (FieldConstants.kBlueHUB.getX() - FieldConstants.kHalfHUBLength)  && pose.getX() < (FieldConstants.kBlueHUB.getX() + FieldConstants.kHalfHUBLength)) {

      // If inside upper trench
      if ((pose.getY() > (FieldConstants.kBlueHUB.getY() + FieldConstants.kBumpLength)) && pose.getY() > (FieldConstants.kBlueHUB.getY() + FieldConstants.kBumpLength + 12)) {
        return false;
      }

      // If inside HUB
      if (pose.getY() > (FieldConstants.kBlueHUB.getY() - DriveConstants.kHalfDriveBaseLength) && pose.getY() < (FieldConstants.kBlueHUB.getY() + DriveConstants.kHalfDriveBaseLength)) {
        return false;
      }

      // If inside lower trench
      if ((pose.getY() > (FieldConstants.kBlueHUB.getY() - FieldConstants.kBumpLength)) && pose.getY() > (FieldConstants.kBlueHUB.getY() - FieldConstants.kBumpLength - 12)) {
        return false;
      }
    }

    // If X is inside red field components
    if (pose.getX() > (FieldConstants.kRedHUB.getX() - FieldConstants.kHalfHUBLength)  && pose.getX() < (FieldConstants.kRedHUB.getX() + FieldConstants.kHalfHUBLength)) {

      // If inside upper trench
      if ((pose.getY() > (FieldConstants.kRedHUB.getY() + FieldConstants.kBumpLength)) && pose.getY() > (FieldConstants.kRedHUB.getY() + FieldConstants.kBumpLength + 12)) {
        return false;
      }

      // If inside HUB
      if (pose.getY() > (FieldConstants.kRedHUB.getY() - DriveConstants.kHalfDriveBaseLength) && pose.getY() < (FieldConstants.kRedHUB.getY() + DriveConstants.kHalfDriveBaseLength)) {
        return false;
      }

      // If inside lower trench
      if ((pose.getY() > (FieldConstants.kRedHUB.getY() - FieldConstants.kBumpLength)) && pose.getY() > (FieldConstants.kRedHUB.getY() - FieldConstants.kBumpLength - 12)) {
        return false;
      }
    }
    */
  }

  public void enableLLReading() {
    isAddingToList = true;
  }


  public void disableLLReading() {
    isAddingToList = false;
  }


  public void updateLLAverages() {
    if (llReadings.size() < 2) {
        SmartDashboard.putNumber("X stdev", 0);
        SmartDashboard.putNumber("Y stdev", 0);
        return;
    }


    int n = llReadings.size();
    double sum = 0;


    for (Pose2d pose : llReadings) {
        sum += pose.getX();
    }


    double avg = sum / n;
    double sumOfDiffsSquared = 0;


    for (Pose2d pose : llReadings) {
        double diff = pose.getX() - avg;
        sumOfDiffsSquared += diff * diff;
    }


    double stdev = Math.sqrt(sumOfDiffsSquared / (n - 1));
   
    SmartDashboard.putNumber("X stdev", stdev);


    sum = 0;


    for (Pose2d pose : llReadings) {
        sum += pose.getY();
    }


    avg = sum / n;
    sumOfDiffsSquared = 0;


    for (Pose2d pose : llReadings) {
        double diff = pose.getY() - avg;
        sumOfDiffsSquared += diff * diff;
    }


    stdev = Math.sqrt(sumOfDiffsSquared / (n - 1));
   
    SmartDashboard.putNumber("Y stdev", stdev);
    SmartDashboard.putNumber("Samples", llReadings.size());
}

public void clearLLAverages() {
  while (llReadings.size() > 0) {
    llReadings.remove(0);
  }
}


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetPoseRotation() {
    m_poseEstimator.resetRotation(
      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red 
      ? Rotation2d.fromDegrees(180) 
      : Rotation2d.fromDegrees(0)
    );
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(this.getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

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
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    
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

  public void stop()
  {
    drive(0,0,0,true);
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

  public void driveRobotRelative(ChassisSpeeds speeds) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
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

    builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getState().angle.getRadians(), null);
    builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getVelocity(), null);

    builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getState().angle.getRadians(), null);
    builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getVelocity(), null);

    builder.addDoubleProperty("Back Left Angle", () -> m_rearLeft.getState().angle.getRadians(), null);
    builder.addDoubleProperty("Back Left Velocity", () -> m_rearLeft.getVelocity(), null);

    builder.addDoubleProperty("Back Right Angle", () -> m_rearRight.getState().angle.getRadians(), null);
    builder.addDoubleProperty("Back Right Velocity", () -> m_rearRight.getVelocity(), null);

    builder.addDoubleProperty("Robot Angle", () -> getPose().getRotation().getRadians(), null);
  }
}
