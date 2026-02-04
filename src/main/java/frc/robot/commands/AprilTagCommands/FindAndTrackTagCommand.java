// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AprilTagCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;


public final class FindAndTrackTagCommand extends SequentialCommandGroup{
  public FindAndTrackTagCommand(DriveSubsystem m_drive)
  {
    addCommands(new FindAprilTagCommand(m_drive), new AprilTagTrackAndMoveCommand(m_drive));
  }
}
