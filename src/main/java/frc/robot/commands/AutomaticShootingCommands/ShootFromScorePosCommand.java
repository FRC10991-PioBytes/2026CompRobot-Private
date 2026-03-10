package frc.robot.commands.AutomaticShootingCommands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants.RedScoringPosition;
import frc.robot.Constants.FieldConstants.BlueScoringPosition;
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
                /*
                ArrayList<Pose2d> closestScoringPoses = new ArrayList<>();
                closestScoringPoses.add(new RedScoringPosition(14.467, 2.868, 4000).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(14.765, 2.045, 4200).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(13.474, 1.701, 4000).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(15.057, 0.834, 5600).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(15.351, 2.112, 4700).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(13.871, 1.162, 4200).getPose2d());

                closestScoringPoses.add(new RedScoringPosition(14.721, 4.0345, 4000).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(13.75, 4.0345, 3500).getPose2d());

                closestScoringPoses.add(new RedScoringPosition(14.48, 5.173, 4000).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(14.781, 6, 4200).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(13.458, 6.378, 4000).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(15.041, 7.25, 5600).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(15.366, 5.93, 4700).getPose2d());
                closestScoringPoses.add(new RedScoringPosition(13.871, 6.907, 4200).getPose2d());
                */
                closestScoringPose = m_drive.getPose().nearest(FieldConstants.getRedScoringPoses());
                rpm = FieldConstants.BlueScoringPosition.getNearestRPM(closestScoringPose);
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