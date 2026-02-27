package frc.robot.commands.AutomaticShootingCommands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import java.util.Set;

public class MoveToScorePosCommand extends Command {
    private final DriveSubsystem m_drive;

    // PID Gains
    private final PIDController m_rangePID = new PIDController(0.5, 0, 0); 
    private final PIDController m_rotPID = new PIDController(0.015, 0, 0);

    // Allowed Blue side AprilTag IDs for the HUB
    private final Set<Double> VALID_TAG_IDS = Set.of(24.0, 25.0, 26.0, 27.0);

    private final int MAX_LOST_FRAMES = 10;
    private int m_lostFrames = 0;

    private double lastX = 0;
    private double lastY = 0;
    private double lastRot = 0;

    private Pose2d closestScoringPose;

    private boolean atScoringPose;
    private boolean foundDriverAlliance;

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
        
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            closestScoringPose = m_drive.getPose().nearest(FieldConstants.BlueScoringPosition.getBlueScoringPoses());
        }
        else if (DriverStation.getAlliance().get() == Alliance.Red) {
            closestScoringPose = m_drive.getPose().nearest(FieldConstants.RedScoringPosition.getRedScoringPoses());
        }
        else {
            foundDriverAlliance = true;
        }

        m_drive.getField().getObject("TargetPose").setPose(closestScoringPose);
    }

    @Override
    public void execute() {

        Pose2d robotPose = m_drive.getPose();

        atScoringPose = m_rangePID.atSetpoint() && m_rotPID.atSetpoint();
        SmartDashboard.putBoolean("Game Info/At Scoring Pose", atScoringPose);
        
        double xSpeed = 0;
        double ySpeed = 0;
        double rotSpeed = 0;
        // We only move if: 
        // A) The robot is on the blue side of the field
        if (robotPose.getX() < FieldConstants.kBlueHUB.getX() || robotPose.getX() > FieldConstants.kRedHUB.getX()) {

            Pose2d closestScoringPose = robotPose.nearest(FieldConstants.BlueScoringPosition.getBlueScoringPoses());
            
            // ---------------------------------------------------------
            // FIELD-CENTRIC VECTOR MATH
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
                    approachSpeed = m_rangePID.calculate(distanceToScoringPosition, 0);
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
            m_drive.drive(xSpeed, ySpeed, rotSpeed, 1, false);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return foundDriverAlliance;
    }
}