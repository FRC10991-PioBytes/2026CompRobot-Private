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
  
  private final SparkMax m_leftMotor;
  private final SparkMax m_rightMotor;

  private final SparkClosedLoopController m_leftController;

  private final AbsoluteEncoder m_leftEncoder;

  private Rotation2d m_desiredAngle = new Rotation2d(0);

  /** Creates a new DriveSubsystem. */
  public IntakeSubsystem() {
    m_leftMotor = new SparkMax(IntakeConstants.kLeftPivotCanId, MotorType.kBrushless);
    m_rightMotor = new SparkMax(IntakeConstants.kRightPivotCanId, MotorType.kBrushless);

    m_leftEncoder = m_leftMotor.getAbsoluteEncoder();

    m_leftController = m_leftMotor.getClosedLoopController();

    m_leftMotor.configure(Configs.Intake.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightMotor.configure(Configs.Intake.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_desiredAngle = new Rotation2d(m_leftEncoder.getPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Current Angle (Deg)", getRotation2d().getDegrees());
    SmartDashboard.putNumber("Intake/Setpoint (Deg)", m_desiredAngle.getDegrees());
    SmartDashboard.putNumber("Intake/Applied Output", m_leftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/Output Current", m_leftMotor.getOutputCurrent());
  }

  public Rotation2d getRotation2d()
  {
    return new Rotation2d(m_leftEncoder.getPosition());
  }

  
  public void setPosition(Rotation2d targetAngle)
  {
    m_leftController.setSetpoint(targetAngle.getRadians(), ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
    //m_leftController.setReference(0, null);
    m_desiredAngle = new Rotation2d(targetAngle.getRadians());
  }
}
