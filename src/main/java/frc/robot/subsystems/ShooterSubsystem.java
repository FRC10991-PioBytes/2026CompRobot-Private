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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Configs.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_leaderMotor;
  private final SparkMax m_leftMotor;
  private final SparkMax m_rightMotor;

  private SparkClosedLoopController m_leaderController;

  private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();

  private double m_p = 0.00005;
  private double m_d = 0.00001;
  private double m_ff = 0.000174;//1.0 / 5676; //Neo V1.1 free speed
  private double m_targetRPM = 0;


  /** Creates a new DriveSubsystem. */
  public ShooterSubsystem() {
    
    m_leaderMotor = new SparkMax(ShooterConstants.kLeaderShooterCanId, MotorType.kBrushless);
    m_leftMotor = new SparkMax(ShooterConstants.kLeftShooterCanId, MotorType.kBrushless);
    m_rightMotor = new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

    m_leaderController = m_leaderMotor.getClosedLoopController();

    m_leaderConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40)
      .voltageCompensation(12)
      .closedLoopRampRate(0.5)
      .closedLoop.outputRange(-1, 1);

    configureMotors();

    SmartDashboard.putNumber("Shooter/P", m_p);
    SmartDashboard.putNumber("Shooter/D", m_d);
    SmartDashboard.putNumber("Shooter/Feed Forward", m_ff);
    SmartDashboard.putNumber("Shooter/Target RPM", 0);
  }

  private void configureMotors() {
    // Apply to hardware (Reset to factory defaults first to clear old junk)
    m_leaderMotor.configure(Shooter.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_leftMotor.configure(Shooter.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(Shooter.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVelocity(double rpm)
  {
    if (rpm != m_targetRPM)
    {
      m_targetRPM = rpm;
      //m_leaderController.setReference(m_targetRPM, ControlType.kVelocity);
      m_leaderController.setSetpoint(m_targetRPM, ControlType.kMAXMotionVelocityControl);
    }
    
  }

  public void stop()
  {
    m_leaderMotor.stopMotor();
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

    SmartDashboard.putNumber("Shooter/Actual RPM: ", getActualVelocity());
    SmartDashboard.putNumber("Shooter/Applied Output: ", m_leaderMotor.getAppliedOutput());

    updateTunables();

  }

  private void updateTunables() {
    double readP = SmartDashboard.getNumber("Shooter/P", m_p);
    double readD = SmartDashboard.getNumber("Shooter/D", m_d);
    double readFF = SmartDashboard.getNumber("Shooter/Feed Forward", m_ff);
    double readTarget = SmartDashboard.getNumber("Shooter/Target RPM", m_targetRPM);

    // check if PID constants changed
    if (readP != m_p || readD != m_d || readFF != m_ff) {
      m_p = readP;
      m_d = readD;
      m_ff = readFF;

      // Update the local config object
      m_leaderConfig.closedLoop
          .p(m_p)
          .d(m_d)
          .feedForward.kV(m_ff);

      // Apply ALL changes at once (Batch update)
      // Use kNoResetSafeParameters so we don't wipe the current limit/coast mode
      m_leaderMotor.configure(m_leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // check if Target Velocity changed via dashboard
    if (readTarget != m_targetRPM) {
      setVelocity(readTarget);
    }
  }

}
