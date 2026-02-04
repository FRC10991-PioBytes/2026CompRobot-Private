// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;


public final class ExtendThenIntakeInCommand extends Command {
  private IntakeSubsystem m_intake;
  private Timer m_timer;

  public ExtendThenIntakeInCommand(IntakeSubsystem intake)
  {
    m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setPivotPosition(Constants.IntakeConstants.kIntakeExtendedEncoderPosition);

    if (m_timer.hasElapsed(1))
    {
      m_intake.runRollers(Constants.IntakeConstants.kIntakeInSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopRollers();
    m_timer.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
