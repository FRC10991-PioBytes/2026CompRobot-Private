// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  private final SparkMax m_leftRollerMotor;
  private final SparkMax m_rightRollerMotor;

  private final SparkMax m_leftPivotMotor;
  private final SparkMax m_rightPivotMotor;

  private final SparkClosedLoopController m_leftController;

  private final AbsoluteEncoder m_leftEncoder;

  private Rotation2d m_desiredAngle = new Rotation2d(0);
  private double kArmGravityVoltage = 0;

  /** Creates a new DriveSubsystem. */
  public IntakeSubsystem() {
    // Roller motors
    m_leftRollerMotor = new SparkMax(IntakeConstants.kLeftRollerCanId, MotorType.kBrushless);
    m_rightRollerMotor = new SparkMax(IntakeConstants.kRightRollerCanId, MotorType.kBrushless);
    
    m_leftRollerMotor.configure(Configs.Intake.leftRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightRollerMotor.configure(Configs.Intake.rightRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Pivot motors
    m_leftPivotMotor = new SparkMax(IntakeConstants.kLeftPivotCanId, MotorType.kBrushless);
    m_rightPivotMotor = new SparkMax(IntakeConstants.kRightPivotCanId, MotorType.kBrushless);

    m_leftEncoder = m_leftPivotMotor.getAbsoluteEncoder();

    m_leftController = m_leftPivotMotor.getClosedLoopController();

    m_leftPivotMotor.configure(Configs.Intake.leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightPivotMotor.configure(Configs.Intake.rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_desiredAngle = new Rotation2d(m_leftEncoder.getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Current Angle (Deg)", getPivotRotation2d().getDegrees());
    SmartDashboard.putNumber("Intake/Setpoint (Deg)", m_desiredAngle.getDegrees());
    SmartDashboard.putNumber("Intake/Current Angle (Rad)", getPivotRotation2d().getRadians());
    SmartDashboard.putNumber("Intake/Setpoint (Rad)", m_desiredAngle.getRadians());
    SmartDashboard.putNumber("Intake/Pivot Applied Output", m_leftPivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/Pivot Output Current", m_leftPivotMotor.getOutputCurrent());
  }

  public void runRollers(double speed)
  {
    if (Math.abs(speed) < 1)
    {
      m_leftRollerMotor.set(speed);
    }
  }

  public void stopRollers()
  {
    m_leftRollerMotor.stopMotor();
  }

  public Rotation2d getPivotRotation2d()
  {
    return new Rotation2d(m_leftEncoder.getPosition());
  }

  
  public void setPivotPosition(Rotation2d targetAngle)
  {
    m_desiredAngle = new Rotation2d(targetAngle.getRadians());
    double targetRadians = targetAngle.getRadians();

    double feedForwardVolts = kArmGravityVoltage * Math.cos(targetRadians);

    m_leftController.setSetpoint(targetAngle.getRadians(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedForwardVolts);
  }

  public void stopPivot()
  {
    m_leftPivotMotor.stopMotor();
  }
}
