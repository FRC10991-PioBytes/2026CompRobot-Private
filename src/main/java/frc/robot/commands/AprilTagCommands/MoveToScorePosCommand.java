package frc.robot.commands.AprilTagCommands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import java.util.Set;

public class MoveToScorePosCommand extends Command {
    private final DriveSubsystem m_drive;

    // PID Gains
    private final PIDController m_rangePID = new PIDController(0, 0, 0); 
    private final PIDController m_rotPID = new PIDController(0, 0, 0);

    // Allowed Blue side AprilTag IDs for the HUB
    private final Set<Double> VALID_TAG_IDS = Set.of(24.0, 25.0, 26.0, 27.0);

    private final int MAX_LOST_FRAMES = 10;
    private int m_lostFrames = 0;

    private double lastX = 0;
    private double lastY = 0;
    private double lastRot = 0;

    private Pose2d closestScoringPose = null;

    private final Field2d field = new Field2d();

    public MoveToScorePosCommand(DriveSubsystem drive) {
        m_drive = drive;
        addRequirements(m_drive);
        
        m_rangePID.setTolerance(0.1);

        m_rotPID.enableContinuousInput(-180, 180);
        m_rotPID.setTolerance(1.0); 

        
    }

    public void initialize() {
        m_rangePID.reset();
        m_rotPID.reset();

        while (closestScoringPose == null)
        {
            closestScoringPose = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").pose
                .nearest(FieldConstants.BlueScoringPosition.getBlueScoringPoses());
        }

        field.getObject("TargetPose").setPose(closestScoringPose);
    }

    @Override
    public void execute() {
        // 1. Update Limelight with Gyro (Crucial for MegaTag 2)
        double gyroYaw = m_drive.getHeading(); 
        LimelightHelpers.SetRobotOrientation("limelight", gyroYaw, 0, 0, 0, 0, 0);
        
        /*
        // 2. Get Full Results to check all visible tags
        //LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight");
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
        */

        // 3. Get the MegaTag 2 Pose (uses ALL visible tags for best accuracy)
        // We perform this check separately from the ID check to ensure we get the best pose data
        LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        boolean hasValidHubTag = true;

        double xSpeed = 0;
        double ySpeed = 0;
        double rotSpeed = 0;
        // We only move if: 
        // A) The Limelight actually has a pose estimate
        // B) One of the tags contributing to the scene is a HUB tag (24-27)
        // C) The robot is on the blue side of the field
        if (mt2Estimate.tagCount > 0 && hasValidHubTag && mt2Estimate.pose.getX() < FieldConstants.kBlueHUB.getX()) {
            m_lostFrames = 0;
            
            Pose2d robotPose = mt2Estimate.pose;
            field.setRobotPose(robotPose);
            SmartDashboard.putData("Game Info/mt2field", field);

            Pose2d closestScoringPose = robotPose.nearest(FieldConstants.BlueScoringPosition.getBlueScoringPoses());

            // ---------------------------------------------------------
            // 4. FIELD-CENTRIC VECTOR MATH
            // ---------------------------------------------------------

            // Vector from robot to scoring position
            Translation2d robotToScoringPositionVector = closestScoringPose.getTranslation().minus(robotPose.getTranslation());

            // Distance to scoring position
            double distanceToScoringPosition = robotToScoringPositionVector.getNorm();
            if (distanceToScoringPosition != 0) {

                double approachSpeed = 0;

                // Range speed
                if (!m_rangePID.atSetpoint()) 
                {
                    // Positive value from PID = move backwards
                    approachSpeed = -m_rangePID.calculate(distanceToScoringPosition, 0);
                }

                // Rotation Speed
                if (!m_rotPID.atSetpoint()) 
                {
                    rotSpeed = m_rotPID.calculate(robotPose.getRotation().getDegrees(), closestScoringPose.getRotation().getDegrees());
                }

                // 6. Calculate Field-Centric Velocities
                // approachSpeed is the MAGNITUDE. 
                
                Translation2d adjustedRobotToHubVector = robotToScoringPositionVector.div(distanceToScoringPosition).times(approachSpeed);
                xSpeed = adjustedRobotToHubVector.getX();
                ySpeed = adjustedRobotToHubVector.getY();
            }

            lastX = xSpeed;
            lastY = ySpeed;
            lastRot = rotSpeed;

            // 7. Drive Command
            // fieldRelative = true, so these X/Y values are treated as Field X/Y
            m_drive.drive(xSpeed, ySpeed, rotSpeed, 1, true);
        }
        else {
            m_lostFrames++;
            if (m_lostFrames > MAX_LOST_FRAMES) {
                m_drive.drive(0, 0, 0, 0, true);
                lastX = 0;
                lastY = 0;
                lastRot = 0;
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