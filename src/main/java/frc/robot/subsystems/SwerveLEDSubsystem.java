// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;


public class SwerveLEDSubsystem extends SubsystemBase {
  
  private boolean isBlue;
  private LEDState m_currentState;
  private final int m_length = 96;
  private final AddressableLED m_ledStrip;
  private final AddressableLEDBuffer m_ledBuffer;

  private CircularLEDBuffer m_frontLeft;
  private CircularLEDBuffer m_frontRight;
  private CircularLEDBuffer m_rearLeft;
  private CircularLEDBuffer m_rearRight;

  private CircularLEDBuffer[] swerveLEDs;

  private DriveSubsystem m_drive;

  public enum LEDState {
    Off,
    Loading,
    Azimuth,
    LookingForTarget,
    TargetFound,
  }

  public SwerveLEDSubsystem(DriveSubsystem drive) {

    isBlue = true;
    m_currentState = LEDState.Off;
    
    m_ledStrip = new AddressableLED(LEDConstants.kLEDStripPort);
    m_ledBuffer = new AddressableLEDBuffer(m_length);

    AddressableLEDBufferView frontLeftView = m_ledBuffer.createView(0, m_length / 4 - 1);
    m_frontLeft = new CircularLEDBuffer(frontLeftView, LEDConstants.kFrontLeftLEDAngularOffset);

    AddressableLEDBufferView frontRightView = m_ledBuffer.createView(m_length / 4 - 1, m_length / 2 - 1);
    m_frontRight = new CircularLEDBuffer(frontRightView, LEDConstants.kFrontRightLEDAngularOffset);

    AddressableLEDBufferView rearLeftView = m_ledBuffer.createView(m_length / 2 - 1, 3 * m_length / 4 - 1);
    m_rearLeft = new CircularLEDBuffer(rearLeftView, LEDConstants.kBackLeftLEDAngularOffset);

    AddressableLEDBufferView rearRightView = m_ledBuffer.createView(3 * m_length / 4 - 1, m_length - 1);
    m_rearRight = new CircularLEDBuffer(rearRightView, LEDConstants.kBackRightEDAngularOffset);

    swerveLEDs = new CircularLEDBuffer[] {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};

    
    m_ledStrip.start();
  }

  @Override
  public void periodic() {

    isBlue = DriverStation.getAlliance().get() == Alliance.Blue;

    switch (m_currentState) {
      case Off:
      {
        setOff();
        break;
      }
      case Loading:
      {
        setLoadingPattern();
        break;
      }
      case Azimuth:
      {
        setAzimuthPattern(() -> m_drive.getModuleStates());
        break;
      }
      case LookingForTarget:
      {
        setLookingForTargetPattern();
        break;
      }
      case TargetFound:
      {
        setTargetFoundPattern();
        break;
      }
    }

    updateLEDStrip();
  }

  public void updateLEDStrip() {
    m_ledStrip.setData(m_ledBuffer);
  }

  public void setLookingForTargetPattern() {
    for (int i = 0; i < swerveLEDs.length; i++) {
      swerveLEDs[i].setLookingForTargetPattern();
    }
  }

  public void setTargetFoundPattern() {
    for (int i = 0; i < swerveLEDs.length; i++) {
      swerveLEDs[i].setTargetFoundPattern();
    }
  }

  public void setLoadingPattern() {
    for (int i = 0; i < swerveLEDs.length; i++) {
      swerveLEDs[i].setLoadingPattern();
    }
  }

  // Sets the Azimuth Pattern given a wheel direction
  public void setAzimuthPattern(Supplier<SwerveModuleState[]> currentStates) {
    var states = currentStates.get();
    for (int i = 0; i < swerveLEDs.length; i++) {
      swerveLEDs[i].setAzimuthPattern(states[i].angle);
    }
  }

  public void setOff() {
    for (int i = 0; i < swerveLEDs.length; i++) {
      swerveLEDs[i].setOff();
    }
  }
}