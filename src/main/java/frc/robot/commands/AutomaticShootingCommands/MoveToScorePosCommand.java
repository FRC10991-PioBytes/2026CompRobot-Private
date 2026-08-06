package frc.robot.commands.AutomaticShootingCommands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveLEDSubsystem;
import frc.robot.subsystems.SwerveLEDSubsystem.LEDState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;

public class MoveToScorePosCommand extends Command {
    private final DriveSubsystem m_drive;
    private final SwerveLEDSubsystem m_leds;

    // PID Gains
    private final PIDController m_rangePID = new PIDController(2, 0, 0); 
    private final PIDController m_rotPID = new PIDController(0.035, 0, 0);

    private Pose2d closestScoringPose;

    private boolean atScoringPose;
    private boolean noDriverAllianceFound;

    public MoveToScorePosCommand(DriveSubsystem drive, SwerveLEDSubsystem ledSubsystem) {
        m_drive = drive;
        m_leds = ledSubsystem;

        addRequirements(m_drive, m_leds);
        
        m_rangePID.setTolerance(0.01);

        m_rotPID.enableContinuousInput(-180, 180);
        m_rotPID.setTolerance(0.5); 
    }

    public void initialize() {
        m_rangePID.reset();
        m_rotPID.reset();
        
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            closestScoringPose = m_drive.getPose().nearest(FieldConstants.getBlueScoringPoses());
        }
        else if (DriverStation.getAlliance().get() == Alliance.Red) {
            closestScoringPose = m_drive.getPose().nearest(FieldConstants.getRedScoringPoses());
        }
        else {
            noDriverAllianceFound = true;
        }

        m_drive.getField().getObject("TargetPose").setPose(closestScoringPose);
    }

    @Override
    public void execute() {

        Pose2d robotPose = m_drive.getPose();

        atScoringPose = m_rangePID.atSetpoint() && m_rotPID.atSetpoint();
        SmartDashboard.putBoolean("Field Info/At Scoring Pose", atScoringPose);
        
        double xSpeed = 0;
        double ySpeed = 0;
        double rotSpeed = 0;

        if (robotPose.getX() < FieldConstants.kBlueHUB.getX() || robotPose.getX() > FieldConstants.kRedHUB.getX()) {
            
            // ---------------------------------------------------------
            // FIELD-CENTRIC VECTOR MATH
            // ---------------------------------------------------------

            // Vector from robot to scoring position
            Translation2d robotToScoringPositionVector = closestScoringPose.getTranslation().minus(robotPose.getTranslation());
            SmartDashboard.putNumber("Field Info/X Robot to Scoring Pos", robotToScoringPositionVector.getX());
            SmartDashboard.putNumber("Field Info/Y Robot to Scoring Pos", robotToScoringPositionVector.getY());

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

            SmartDashboard.putNumber("X speed", xSpeed);
            SmartDashboard.putNumber("Y speed", ySpeed);
            SmartDashboard.putNumber("Rotation", rotSpeed);
            // 7. Drive Command
            // fieldRelative = true, so these X/Y values are treated as Field X/Y
            if (atScoringPose)
            {
                m_leds.setState(LEDState.TargetFound);
                m_drive.setX();
            }
            else
            {
                m_leds.setState(LEDState.LookingForTarget);
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    m_drive.drive(xSpeed, ySpeed, rotSpeed, true);
                }
                else {
                     m_drive.drive(-xSpeed, -ySpeed, rotSpeed, true);
                }
                
            }
            
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return noDriverAllianceFound;
    }
}