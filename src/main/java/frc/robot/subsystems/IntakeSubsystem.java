// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  private final SparkMax m_rollerMotor;

  private final SparkMax m_pivotMotor;
  private SparkClosedLoopController m_pivotController;

  private final RelativeEncoder m_pivotEncoder;
  private Rotation2d m_targetAngle;
  private double kV = 0;

  /** Creates a new DriveSubsystem. */
  public IntakeSubsystem() {
    // Roller motor
    m_rollerMotor = new SparkMax(IntakeConstants.kLeaderRollerCanId, MotorType.kBrushless);
    
    m_rollerMotor.configure(Configs.Intake.leaderRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Pivot motor
    m_pivotMotor = new SparkMax(IntakeConstants.kLeaderPivotCanId, MotorType.kBrushless);
    m_pivotController = m_pivotMotor.getClosedLoopController();

    m_pivotMotor.configure(Configs.Intake.leaderPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_pivotEncoder = m_pivotMotor.getEncoder();
    m_pivotEncoder.setPosition(IntakeConstants.kIntakeOffset.getRotations());
    m_targetAngle = IntakeConstants.kIntakeOffset;
    SmartDashboard.putNumber("KV", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Intake", this);
    double newKV = SmartDashboard.getNumber("KV", kV);

    if (newKV != kV) {
      SparkMaxConfig config = new SparkMaxConfig().apply(Configs.Intake.leaderPivotConfig);
      config.closedLoop.feedForward.kV(newKV);
      m_pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      kV = newKV;
      System.out.println("Yo");
    }
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
    return Rotation2d.fromRotations(m_pivotEncoder.getPosition());
  }

  public void setPivotPosition(Rotation2d angle) {
    m_pivotController.setSetpoint(angle.getRotations(), ControlType.kMAXMotionPositionControl);
    m_targetAngle = angle;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Current Angle (Deg)", () -> getPivotRotation2d().getDegrees(), null);

    builder.addDoubleProperty("Pivot Duty Cycle", () -> m_pivotMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("Pivot Output Current", () -> m_pivotMotor.getOutputCurrent(), null);

    builder.addDoubleProperty("Roller Duty Cycle", () -> m_rollerMotor.getAppliedOutput(), null);
    builder.addDoubleProperty("Roller Output Current", () -> m_rollerMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Roller Velocity", () -> m_rollerMotor.getEncoder().getVelocity(), null);

    builder.addDoubleProperty("Encoder value", () -> getPivotRotation2d().getRotations(), null);
    builder.addDoubleProperty("Encoder setpoint (Deg)", () -> m_targetAngle.getDegrees(), null);
  }

}
