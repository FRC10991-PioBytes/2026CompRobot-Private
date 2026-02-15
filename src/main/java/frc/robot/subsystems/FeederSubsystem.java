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
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.FeederConstants;
import frc.robot.Configs.Feeder;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax m_leaderMotor;

  private SparkClosedLoopController m_leaderController;

  private SparkMaxConfig m_leaderConfig = new SparkMaxConfig();

  private double m_p = 0;
  private double m_d = 0;
  private double m_ff = 0;
  private double m_targetRPM = 0;

  private final MutVoltage m_appliedVoltage = new MutVoltage(0.0, 0.0, Units.Volts);
  private final MutAngle m_angle = new MutAngle(0, 0, Units.Revolutions); // Revolutions
  private final MutAngularVelocity m_velocity = new MutAngularVelocity(0, 0, Units.Revolutions.per(Units.Minute)); // RPM
  private final SysIdRoutine m_sysIdRoutine;


  /** Creates a new DriveSubsystem. */
  public FeederSubsystem() {
    
    m_leaderMotor = new SparkMax(FeederConstants.kLeftFeederCanId, MotorType.kBrushless);

    m_leaderController = m_leaderMotor.getClosedLoopController();

    m_leaderMotor.configure(Feeder.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Feeder/P", m_p);
    SmartDashboard.putNumber("Feeder/D", m_d);
    SmartDashboard.putNumber("Feeder/Feed Forward", m_ff);
    SmartDashboard.putNumber("Feeder/Target RPM", 0);

    m_sysIdRoutine =
      new SysIdRoutine(
        // Config
        new SysIdRoutine.Config(),

        // Mechanism
        new SysIdRoutine.Mechanism(
          (voltage) -> m_leaderMotor.setVoltage(voltage),
          (log) -> {
            log.motor("shooter")
              .voltage(
                m_appliedVoltage.mut_replace(
                  m_leaderMotor.getAppliedOutput() * RobotController.getBatteryVoltage(), Units.Volts
                )
              )
              .angularPosition(
                m_angle.mut_replace(m_leaderMotor.getEncoder().getPosition(), Units.Revolutions)
              )
              .angularVelocity(
                m_velocity.mut_replace(m_leaderMotor.getEncoder().getVelocity(), Units.Revolutions.per(Units.Minute))
              );
          },
          this
        )
      );
  }

  public void setVelocity(double rpm)
  {
    if (rpm != m_targetRPM)
    {
      m_targetRPM = rpm;
      m_leaderController.setSetpoint(m_targetRPM, ControlType.kMAXMotionVelocityControl);
    }
    
  }

  public void stop()
  {
    m_targetRPM = 0;
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

    SmartDashboard.putNumber("Feeder/Actual RPM: ", getActualVelocity());
    SmartDashboard.putNumber("Feeder/Applied Output: ", m_leaderMotor.getAppliedOutput());

    //updateTunables();

  }

  private void updateTunables() {
    double readP = SmartDashboard.getNumber("Feeder/P", m_p);
    double readD = SmartDashboard.getNumber("Feeder/D", m_d);
    double readFF = SmartDashboard.getNumber("Feeder/Feed Forward", m_ff);
    double readTarget = SmartDashboard.getNumber("Feeder/Target RPM", m_targetRPM);

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
      System.out.println("Feeder Target set -------------------------------------------------------");
      setVelocity(readTarget);
    }
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
  
}
