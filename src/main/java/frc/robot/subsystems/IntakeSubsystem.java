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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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

  private SparkMaxConfig m_leftPivotConfig = new SparkMaxConfig();

  private final AbsoluteEncoder m_leftEncoder;

  private Rotation2d m_desiredAngle = new Rotation2d(0);

  private double m_p = 0;
  private double m_d = 0;
  private double m_ff = 0;
  //private double kArmGravityVoltage = 0;

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

    m_leftPivotConfig
      .idleMode(IdleMode.kCoast)
      .smartCurrentLimit(40)
      .voltageCompensation(12)
      .closedLoopRampRate(0.5)
      .closedLoop.outputRange(-1, 1);

    m_leftPivotConfig.absoluteEncoder
      .positionConversionFactor(2 * Math.PI)
      .velocityConversionFactor(2 * Math.PI / 60);

    m_leftPivotMotor.configure(Configs.Intake.leftPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_rightPivotMotor.configure(Configs.Intake.rightPivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_desiredAngle = new Rotation2d(m_leftEncoder.getPosition());

    SmartDashboard.putNumber("Intake/Pivot/P", m_p);
    SmartDashboard.putNumber("Intake/Pivot/D", m_d);
    SmartDashboard.putNumber("Intake/Pivot/Feed Forward", m_ff);
    SmartDashboard.putNumber("Intake/Target (Rad)", 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/Current Angle (Deg)", getPivotRotation2d().getDegrees());
    SmartDashboard.putNumber("Intake/Setpoint (Deg)", m_desiredAngle.getDegrees());
    SmartDashboard.putNumber("Intake/Current Angle (Rad)", getPivotRotation2d().getRadians());
    SmartDashboard.putNumber("Intake/Setpoint (Rad)", m_desiredAngle.getRadians());
    SmartDashboard.putNumber("Intake/Pivot Applied Output", m_leftPivotMotor.getAppliedOutput());
    SmartDashboard.putNumber("Intake/Pivot Output Current", m_leftPivotMotor.getOutputCurrent());

    updateTunables();
  }

  private void updateTunables()
  {
    double readP = SmartDashboard.getNumber("Intake/Pivot/P", m_p);
    double readD = SmartDashboard.getNumber("Intake/Pivot/D", m_d);
    double readFF = SmartDashboard.getNumber("Intake/Pivot/Feed Forward", m_ff);
    double readTarget = SmartDashboard.getNumber("Intake/Target (Rad)", 0);

    // check if PID constants changed
    if (readP != m_p || readD != m_d || readFF != m_ff) {
      m_p = readP;
      m_d = readD;
      m_ff = readFF;

      // Update the local config object
      m_leftPivotConfig.closedLoop
          .p(m_p)
          .d(m_d)
          .feedForward.kV(m_ff);

      // Apply ALL changes at once (Batch update)
      // Use kNoResetSafeParameters so we don't wipe the current limit/coast mode
      m_leftPivotMotor.configure(m_leftPivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    if (readTarget != m_desiredAngle.getRadians()) {
      setPivotPosition(new Rotation2d(readTarget));
    }
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
    //double targetRadians = targetAngle.getRadians();

    //double feedForwardVolts = kArmGravityVoltage * Math.cos(targetRadians);

    m_leftController.setSetpoint(targetAngle.getRadians(), ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("Intake/Target (Rad)", targetAngle.getRadians());
  }

  public void stopPivot()
  {
    m_leftPivotMotor.stopMotor();
  }
}
