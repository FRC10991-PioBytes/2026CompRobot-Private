// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;

/** An example command that uses an example subsystem. */
public class TrackAprilTagCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  private final PIDController turnPID = new PIDController(0.02,0,0);

  private final DriveSubsystem m_drive;
  private final double ANGLE_TOLERANCE = 1; //5 degree tolerance
  private final int FRAME_ERROR = 10;
  private int frameCounter = 0;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TrackAprilTagCommand(DriveSubsystem driveSubsystem) {
    m_drive = driveSubsystem;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV(VisionConstants.kLimelightName);
    if (hasTarget) 
    {
      frameCounter = 0;
      double tx = LimelightHelpers.getTX(VisionConstants.kLimelightName);

      if (Math.abs(tx) < ANGLE_TOLERANCE)
      {
        tx = 0;
      }
      
      double rotationSpeed = turnPID.calculate(tx, 0);

      // If positive, turn clockwise
      // If negative, turn counter clockwise
      m_drive.drive(0, 0, rotationSpeed, 1, false);
    }
    else
    {
      frameCounter++;
      if (frameCounter > FRAME_ERROR)
      {
        m_drive.stop();
      }
    }
  }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
