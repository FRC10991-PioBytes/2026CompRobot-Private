package frc.robot.commands.AutomaticShootingCommands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;

public class ShootFromScorePosCommand extends Command {
    private final ShooterSubsystem m_shooter;
    private final DriveSubsystem m_drive;

    private Pose2d closestScoringPose;
    private double rpm;

    public ShootFromScorePosCommand(ShooterSubsystem shooter, DriveSubsystem drive) {
        m_shooter = shooter;
        m_drive = drive;
        addRequirements(m_shooter);
    }

    public void initialize() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            try {
                closestScoringPose = m_drive.getPose().nearest(FieldConstants.getBlueScoringPoses());
                rpm = FieldConstants.BlueScoringPosition.getNearestRPM(closestScoringPose);
            }
            catch (Exception e) {
                e.printStackTrace();
                rpm = 4200;
            }
            
        }
        else if (DriverStation.getAlliance().get() == Alliance.Red) {
            try {
                closestScoringPose = m_drive.getPose().nearest(FieldConstants.getRedScoringPoses());
                rpm = FieldConstants.RedScoringPosition.getNearestRPM(closestScoringPose);
            }
            catch (Exception e) {
                e.printStackTrace();
                rpm = 4200;
            }
        }
        else {
            rpm = 0;
        }
        
        m_shooter.setVelocity(rpm);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}