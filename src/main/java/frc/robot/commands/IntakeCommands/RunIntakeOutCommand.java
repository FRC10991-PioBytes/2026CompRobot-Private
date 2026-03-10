// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;


public final class RunIntakeOutCommand extends Command {
  private IntakeSubsystem m_intake;

  public RunIntakeOutCommand(IntakeSubsystem intake)
  {
    m_intake = intake;

    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Running rollers out");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runRollers(Constants.IntakeConstants.kIntakeOutSpeed);
    SmartDashboard.putBoolean("Intake/Rollers running", true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopRollers();
    System.out.println("Stopping rollers");
    SmartDashboard.putBoolean("Intake/Rollers running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
