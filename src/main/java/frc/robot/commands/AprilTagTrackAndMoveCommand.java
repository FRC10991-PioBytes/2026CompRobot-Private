package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AprilTagTrackAndMoveCommand extends Command {
  private final DriveSubsystem m_drive;
  
  // PID Gains (Tuning is required for your specific robot weight)
  private final PIDController m_rangePID = new PIDController(0.5, 0, 0); 
  private final PIDController m_rotPID = new PIDController(0.02, 0, 0);

  private final double DESIRED_RADIUS = 1.5; // Meters
  private final int MAX_LOST_FRAMES = 10; // ~200ms at 50Hz
  private int m_lostFrames = 0;

  private final Field2d field = new Field2d();

  public AprilTagTrackAndMoveCommand(DriveSubsystem drive) {
      m_drive = drive;
      m_drive.zeroHeading();
      addRequirements(m_drive);
  }

  @Override
  public void execute() {
      // 1. Get the vision-compensated pose
      // MegaTag2 requires the current gyro data to stabilize the vision estimate
      double gyroYaw = m_drive.getHeading(); // Ensure this returns degrees
      double gyroPitch = 0; // Usually 0 unless climbing
      double gyroRoll = 0;
      SmartDashboard.putNumber("gyro yaw", gyroYaw);
      LimelightHelpers.SetRobotOrientation("limelight", gyroYaw, 0, 0, 0, 0, 0);
      
      // We use TargetPose_RobotSpace to get coordinates relative to the robot's front
      double[] poseEntry = LimelightHelpers.getTargetPose_RobotSpace("limelight");
      Pose2d mt2pose = new Pose2d(new Translation2d(poseEntry[0], poseEntry[1]), new Rotation2d(poseEntry[2]));
      field.setRobotPose(mt2pose);
      SmartDashboard.putData("mt2field", field);
      SmartDashboard.putNumber("0", poseEntry[0]);
      SmartDashboard.putNumber("1", poseEntry[1]);
      SmartDashboard.putNumber("2", poseEntry[2]);
      
      boolean hasTarget = LimelightHelpers.getTV("limelight");

      if (hasTarget && poseEntry.length > 0) {
          m_lostFrames = 0; // Reset persistence counter

          double xTag = poseEntry[2]; // Forward/Back meters
          double yTag = poseEntry[0]; // Left/Right meters
          
          // 2. Vector Math for the Arc
          double currentRadius = Math.sqrt(Math.pow(xTag, 2) + Math.pow(yTag, 2));
          double distanceError = currentRadius - DESIRED_RADIUS;
          
          //SmartDashboard.putNumber("Radius", currentRadius);
          if (currentRadius != 0)
          {
            //Only move if we are outside a 5cm deadband
            double speedMagnitude = 0;
            if (Math.abs(distanceError) > 0.05) {
                speedMagnitude = m_rangePID.calculate(currentRadius, DESIRED_RADIUS);
            }

            if (Double.isNaN(speedMagnitude))
            {
                System.out.println("Speed magnitude is NaN");
            }

            // Project the magnitude onto robot-relative X and Y axes
            // This ensures the robot drives directly toward the tag's coordinates
            double xSpeed = (xTag / currentRadius) * speedMagnitude;
            double ySpeed = (yTag / currentRadius) * speedMagnitude;

            // 3. Heading (Rotation)
            // Still using TX for simple 'facing' logic
            double tx = LimelightHelpers.getTX("limelight");
            double rotSpeed = m_rotPID.calculate(tx, 0);

            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);
            SmartDashboard.putNumber("rotSpeed", rotSpeed);
            //m_drive.drive(xSpeed, ySpeed, rotSpeed, 1, false);
            m_drive.drive(-xSpeed, ySpeed, rotSpeed, 1, false);
            }
          else
          {
            System.out.println("Current radius is 0");
          }
          /*
          // Only move if we are outside a 5cm deadband
          double speedMagnitude = 0;
          if (Math.abs(distanceError) > 0.05) {
              speedMagnitude = m_rangePID.calculate(currentRadius, DESIRED_RADIUS);
          }

          if (Double.isNaN(speedMagnitude))
          {
            System.out.println("Speed magnitude is NaN");
          }

          // Project the magnitude onto robot-relative X and Y axes
          // This ensures the robot drives directly toward the tag's coordinates
          double xSpeed = (xTag / currentRadius) * speedMagnitude;
          double ySpeed = (yTag / currentRadius) * speedMagnitude;

          // 3. Heading (Rotation)
          // Still using TX for simple 'facing' logic
          double tx = LimelightHelpers.getTX("limelight");
          double rotSpeed = m_rotPID.calculate(tx, 0);

          SmartDashboard.putNumber("xSpeed", xSpeed);
          SmartDashboard.putNumber("ySpeed", ySpeed);
          SmartDashboard.putNumber("rotSpeed", rotSpeed);
          //m_drive.drive(xSpeed, ySpeed, rotSpeed, 1, false);
          m_drive.drive(-xSpeed, ySpeed, rotSpeed, 1, false);
          */

      } else {
          // 4. Persistence Logic
          m_lostFrames++;
          if (m_lostFrames < MAX_LOST_FRAMES) {
              // Keep doing what we were doing (coasting) or slow down slightly
              // We don't call drive(0,0,0) here yet to prevent jitter
          } else {

              m_drive.drive(0, 0, 0, 1, false);
          }
      }
  }

  @Override
  public void end(boolean interrupted) {
      m_drive.drive(0, 0, 0, 0.1, false);
  }
}