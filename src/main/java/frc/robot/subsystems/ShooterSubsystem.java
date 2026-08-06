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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Configs.Shooter;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_leaderMotor;
  private final SparkMax m_rightMotor;

  private SparkClosedLoopController m_leaderController;

  private double m_targetRPM = 0;

  private double m_gearRatio = (double) 23 / 18;
    

  /** Creates a new DriveSubsystem. */
  public ShooterSubsystem() {
    
    m_leaderMotor = new SparkMax(ShooterConstants.kLeaderShooterCanId, MotorType.kBrushless);
    m_rightMotor = new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

    m_leaderController = m_leaderMotor.getClosedLoopController();

    configureMotors();
    
    setVelocity(ShooterConstants.kShooterIdleVelocity);

  }

  private void configureMotors() {
    // Apply to hardware (Reset to factory defaults first to clear old junk)
    m_leaderMotor.configure(Shooter.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(Shooter.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVelocity(double rpm)
  {
    if (rpm != m_targetRPM)
    {
      m_targetRPM = rpm;
      m_leaderController.setSetpoint(m_targetRPM, ControlType.kMAXMotionVelocityControl);
    }
    System.out.println("Set shooter to " + rpm);
  }

  public void stop()
  {
    m_targetRPM = 0;
    System.out.println("Shooter stopped");
    
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

    SmartDashboard.putData("Shooter", this);
    if (m_targetRPM != 0)
    {
      SmartDashboard.putBoolean("Shooter/Shooter Running", true);

    }
    else
    {
      SmartDashboard.putBoolean("Shooter/Shooter Running", false);
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("Input Shaft RPM", () -> getActualVelocity(), null);
    builder.addDoubleProperty("Output Shaft RPM", () -> getActualVelocity() * m_gearRatio, null);
    builder.addDoubleProperty("Shooter Setpoint", () -> m_targetRPM, null);
    builder.addBooleanProperty("Within 50 RPM", () -> isAtSpeed(50.0), null);

    builder.addDoubleProperty("L Applied Output", () -> m_leaderMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("L Bus Voltage", () -> m_leaderMotor.getBusVoltage(), null);
    builder.addDoubleProperty("L Output Current", () -> m_leaderMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("L Temperature", () -> m_leaderMotor.getMotorTemperature(), null);
    builder.addBooleanProperty("L Faults", () -> m_leaderMotor.getFaults().can, null);

    builder.addDoubleProperty("F Applied Output", () -> m_rightMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("F Bus Voltage", () -> m_rightMotor.getBusVoltage(), null);
    builder.addDoubleProperty("F Output Current", () -> m_rightMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("F Temperature", () -> m_rightMotor.getMotorTemperature(), null);

    builder.addStringProperty("Slot", () -> m_leaderController.getSelectedSlot().toString(), null);

  }
}
