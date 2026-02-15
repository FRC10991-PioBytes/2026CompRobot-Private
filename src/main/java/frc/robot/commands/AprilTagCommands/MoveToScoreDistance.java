package frc.robot.commands.AprilTagCommands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import java.util.Set;

public class MoveToScoreDistance extends Command {
    private final DriveSubsystem m_drive;

    // PID Gains
    private final PIDController m_rangePID = new PIDController(0, 0, 0); 
    private final PIDController m_rotPID = new PIDController(0, 0, 0);

    // Field Coordinate of the HUB Center
    private final Translation2d HUB_CENTER = FieldConstants.kBlueHUB;

    // Allowed Blue side AprilTag IDs for the HUB
    private final Set<Double> VALID_TAG_IDS = Set.of(24.0, 25.0, 26.0, 27.0);

    private final double DESIRED_RADIUS = 2.5; // Meters
    private final int MAX_LOST_FRAMES = 10;
    private int m_lostFrames = 0;

    private double lastX = 0;
    private double lastY = 0;
    private double lastRot = 0;

    private final Field2d field = new Field2d();

    public MoveToScoreDistance(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
        
        m_rangePID.setTolerance(0.1);

        m_rotPID.enableContinuousInput(-180, 180);
        m_rotPID.setTolerance(1.0); 
    }

    @Override
    public void execute() {
        // 1. Update Limelight with Gyro (Crucial for MegaTag 2)
        double gyroYaw = m_drive.getHeading(); 
        LimelightHelpers.SetRobotOrientation("limelight", gyroYaw, 0, 0, 0, 0, 0);
        
        // 2. Get Full Results to check all visible tags
        LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight");
        boolean hasValidHubTag = false;

        // Iterate through ALL tags currently seen by Limelight
        if (results != null && results.targets_Fiducials != null && results.targets_Fiducials.length > 0) {
            for (LimelightHelpers.LimelightTarget_Fiducial tag : results.targets_Fiducials) {
                if (VALID_TAG_IDS.contains(tag.fiducialID)) {
                    hasValidHubTag = true;
                    break; // We found at least one Hub tag, so we are good to go!
                }
            }
        }

        // 3. Get the MegaTag 2 Pose (uses ALL visible tags for best accuracy)
        // We perform this check separately from the ID check to ensure we get the best pose data
        LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        
        double xSpeed = 0;
        double ySpeed = 0;
        double rotSpeed = 0;
        // We only move if: 
        // A) The Limelight actually has a pose estimate
        // B) One of the tags contributing to the scene is a HUB tag (24-27)
        if (mt2Estimate.tagCount > 0 && hasValidHubTag) {
            m_lostFrames = 0;
            
            Pose2d robotPose = mt2Estimate.pose;
            field.setRobotPose(robotPose);
            SmartDashboard.putData("Game Info/mt2field", field);

            // ---------------------------------------------------------
            // 4. FIELD-CENTRIC VECTOR MATH
            // ---------------------------------------------------------

            // Vector from robot to HUB
            Translation2d robotToHubVector = HUB_CENTER.minus(robotPose.getTranslation());

            // Distance to Hub Center
            double distanceToCenter = robotToHubVector.getNorm();
            if (distanceToCenter != 0) {
                // Calculate Angle from Robot -> Hub (Global Field Angle)
                Rotation2d angleToHub = robotToHubVector.getAngle();
                
                // 5. Calculate PID Outputs
                
                double approachSpeed = 0;
                

                // Range speed
                if (!m_rangePID.atSetpoint()) 
                {
                    // Positive value from PID = move backwards
                    approachSpeed = -m_rangePID.calculate(distanceToCenter, DESIRED_RADIUS);
                }

                // Rotation Speed
                if (!m_rotPID.atSetpoint()) 
                {
                    rotSpeed = m_rotPID.calculate(robotPose.getRotation().getDegrees(), angleToHub.getDegrees());
                }

                // 6. Calculate Field-Centric Velocities
                // approachSpeed is the MAGNITUDE. 
                
                Translation2d adjustedRobotToHubVector = robotToHubVector.div(distanceToCenter).times(approachSpeed);
                xSpeed = adjustedRobotToHubVector.getX();
                ySpeed = adjustedRobotToHubVector.getY();
            }

            lastX = xSpeed;
            lastY = ySpeed;
            lastRot = rotSpeed;

            // 7. Drive Command
            // fieldRelative = false, so these X/Y values are treated as Field X/Y
            m_drive.drive(xSpeed, ySpeed, rotSpeed, 1, true);
        }
        else {
            m_lostFrames++;
            if (m_lostFrames > MAX_LOST_FRAMES) {
                m_drive.drive(0, 0, 0, 0, true);
            }
            else {
                m_drive.drive(lastX, lastY, lastRot, 1, true);
            }
        }
    }

    @Override
        public void end(boolean interrupted) {
            m_drive.drive(0, 0, 0, 0, true);
        }
}