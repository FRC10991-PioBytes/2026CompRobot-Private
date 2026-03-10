// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.FeederConstants;
import frc.robot.Configs.Feeder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax m_leaderMotor;
  //private final SparkMax m_agitatorMotor;

  private SparkClosedLoopController m_leaderController;

  private double m_targetRPM = 0;

  /** Creates a new DriveSubsystem. */
  public FeederSubsystem() {
    
    m_leaderMotor = new SparkMax(FeederConstants.kLeftFeederCanId, MotorType.kBrushless);

    m_leaderController = m_leaderMotor.getClosedLoopController();

    m_leaderMotor.configure(Feeder.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //m_agitatorMotor = new SparkMax(FeederConstants.kAgitatorCanId, MotorType.kBrushless);

    //m_agitatorMotor.configure(Feeder.agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Feeder/Target RPM", 0);

    setVelocity(0);
  }

  public void setVelocity(double rpm)
  {
    if (rpm != m_targetRPM)
    {
      m_targetRPM = rpm;
      m_leaderController.setSetpoint(m_targetRPM, ControlType.kMAXMotionVelocityControl);
    }
    
    System.out.println("Setting feeder target rpm to " + rpm);
  }

  /*
  public void runAgitator(double speed)
  {
    m_agitatorMotor.set(speed);
  }
  */

  public void stop()
  {
    m_targetRPM = 0;
    m_leaderMotor.stopMotor();
    //m_agitatorMotor.stopMotor();
    System.out.println("Feeder stopped");
  }

  public boolean isAtSpeed(double tolerance)
  {
    return Math.abs(getActualVelocity() - m_targetRPM) < tolerance;
  }

  public double getActualVelocity()
  {
    return m_leaderMotor.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Feeder/Actual RPM: ", getActualVelocity());
    SmartDashboard.putNumber("Feeder/Applied Output: ", m_leaderMotor.getAppliedOutput());

    if (m_targetRPM != 0)
    {
      SmartDashboard.putBoolean("Feeder/Feeder Running", true);

    }
    else
    {
      SmartDashboard.putBoolean("Feeder/Feeder Running", false);
    }
    /*
    if (m_targetRPM != 0)
    {
      m_agitatorMotor.set(-1);
    }
    else
    {
      m_agitatorMotor.stopMotor();
    }
    */
  }

}
