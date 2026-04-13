// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  private final SparkMax m_rollerMotor;

  private final SparkMax m_pivotMotor;

  private final AbsoluteEncoder m_pivotEncoder;

  /** Creates a new DriveSubsystem. */
  public IntakeSubsystem() {
    // Roller motor
    m_rollerMotor = new SparkMax(IntakeConstants.kLeaderRollerCanId, MotorType.kBrushless);
    
    m_rollerMotor.configure(Configs.Intake.leaderRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Pivot motor
    m_pivotMotor = new SparkMax(IntakeConstants.kLeaderPivotCanId, MotorType.kBrushless);

    m_pivotMotor.configure(Configs.Intake.leaderPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pivotEncoder = m_pivotMotor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
  }

  public void runPivot(double speed) {
    m_pivotMotor.set(speed);
  }

  public void runRollers(double speed) {
      m_rollerMotor.set(speed);
  }

  public void stopRollers() {
    m_rollerMotor.stopMotor();
  }

  public void stopPivot() {
    m_pivotMotor.stopMotor();
  }

  public Rotation2d getPivotRotation2d() {
    return new Rotation2d(m_pivotEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Current Angle (Deg)", () -> getPivotRotation2d().getDegrees(), null);

    builder.addDoubleProperty("Pivot Duty Cycle", () -> m_pivotMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("Pivot Output Current", () -> m_pivotMotor.getOutputCurrent(), null);

    builder.addDoubleProperty("Roller Duty Cycle", () -> m_rollerMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("Roller Output Current", () -> m_rollerMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Roller Velocity", () -> m_rollerMotor.getEncoder().getVelocity(), null);
  }

}
