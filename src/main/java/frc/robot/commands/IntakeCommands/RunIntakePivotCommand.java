// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public final class RunIntakePivotCommand extends Command {
  private IntakeSubsystem m_intake;
  private DoubleSupplier m_power;

  public RunIntakePivotCommand(IntakeSubsystem intake, DoubleSupplier power)
  {
    m_intake = intake;
    m_power = power;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runPivot(m_power.getAsDouble());
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
