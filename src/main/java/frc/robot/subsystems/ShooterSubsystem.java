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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final SparkMax m_leaderMotor;
  private final SparkMax m_rightMotor;

  private SparkClosedLoopController m_leaderController;

  private double m_targetRPM = 0;

  private double m_gearRatio = (double) 23 / 18;

  /*
  private final MutVoltage m_appliedVoltage = new MutVoltage(0.0, 0.0, Units.Volts);
  private final MutAngle m_angle = new MutAngle(0, 0, Units.Revolutions); // Revolutions
  private final MutAngularVelocity m_velocity = new MutAngularVelocity(0, 0, Units.Revolutions.per(Units.Minute)); // RPM
  private final SysIdRoutine m_sysIdRoutine;
  */
    

  /** Creates a new DriveSubsystem. */
  public ShooterSubsystem() {
    
    m_leaderMotor = new SparkMax(ShooterConstants.kLeaderShooterCanId, MotorType.kBrushless);
    m_rightMotor = new SparkMax(ShooterConstants.kRightShooterCanId, MotorType.kBrushless);

    m_leaderController = m_leaderMotor.getClosedLoopController();

    configureMotors();
    
    /*
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
    */
    
    setVelocity(0);

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
    System.out.println("Setting shooter target rpm to " + rpm);
  }

  public void stop()
  {
    m_targetRPM = 0;
    m_leaderMotor.stopMotor();
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
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("Input Shaft RPM", () -> getActualVelocity(), null);
    builder.addDoubleProperty("Output Shaft RPM", () -> getActualVelocity() * m_gearRatio, null);
    builder.addDoubleProperty("Shooter Setpoint", () -> m_targetRPM, null);
    builder.addBooleanProperty("Within 50 RPM", () -> isAtSpeed(50.0), null);

  }
}
