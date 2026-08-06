// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Constants;


public final class SpamIntakePivotCommand extends Command {
  private IntakeSubsystem m_intake;
  private Timer m_timer;

  public SpamIntakePivotCommand(IntakeSubsystem intake)
  {
    m_intake = intake;
    m_timer = new Timer();
    
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = m_timer.get() % 1.5;
    // If even
    if (time < 1) {
      m_intake.runPivot(Constants.IntakeConstants.kPivotSpamUpSpeed);
    }
    else {
      m_intake.runPivot(Constants.IntakeConstants.kPivotSpamDownSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
