package frc.robot.commands.AutomaticShootingCommands;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;

public class FaceToPassCommand extends Command {
    private final DriveSubsystem m_drive;

    private DoubleSupplier m_xSpeed;
    private DoubleSupplier m_ySpeed;
    private Rotation2d m_targetAngle;

    private boolean atAngle;

    private final PIDController m_rotPID = new PIDController(0.035, 0, 0);

    private boolean noDriverAllianceFound;

    public FaceToPassCommand(DriveSubsystem drive, DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
        m_drive = drive;
        addRequirements(m_drive);

        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;

        m_rotPID.enableContinuousInput(-180, 180);
        m_rotPID.setTolerance(1.0);
    }

    public void initialize() {
        m_rotPID.reset();
        
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            m_targetAngle = Rotation2d.fromDegrees(180);
        }
        else if (DriverStation.getAlliance().get() == Alliance.Red) {
            m_targetAngle = Rotation2d.fromDegrees(0);
        }
        else {
            noDriverAllianceFound = true;
        }
    }

    @Override
    public void execute() {

        Pose2d robotPose = m_drive.getPose();
        atAngle = m_rotPID.atSetpoint();

        double rotSpeed = 0;

        // Rotation Speed
        if (!m_rotPID.atSetpoint()) 
        {
            rotSpeed = -m_rotPID.calculate(robotPose.getRotation().getDegrees(), m_targetAngle.getDegrees());
        }

        SmartDashboard.putNumber("rotation", rotSpeed);
        m_drive.drive(m_xSpeed.getAsDouble(), m_ySpeed.getAsDouble(), rotSpeed, true);
        
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