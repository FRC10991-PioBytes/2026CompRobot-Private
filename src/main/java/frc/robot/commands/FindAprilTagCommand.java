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
public class FindAprilTagCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final DriveSubsystem m_drive;
  private final int FRAME_ERROR = 10;
  private int frameCounter;
  private boolean targetFound;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public FindAprilTagCommand(DriveSubsystem driveSubsystem) {
    m_drive = driveSubsystem;

    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    frameCounter = 0;
    targetFound = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean hasTarget = LimelightHelpers.getTV(VisionConstants.kLimelightName);

    if (hasTarget)
    {
      frameCounter++;
      m_drive.drive(0, 0, 1, 0.25, false);
    }
    else
    {
      frameCounter = 0;
      m_drive.drive(0,0,1,0.25,false);
    }

    if (frameCounter > FRAME_ERROR)
    {
      targetFound = true;
    }
  }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targetFound;
  }

}
