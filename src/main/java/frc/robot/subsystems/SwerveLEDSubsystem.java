// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;


public class SwerveLEDSubsystem extends SubsystemBase {
  
  private boolean m_isRed;
  private final int m_length = 96;
  private final AddressableLED m_ledStrip;
  private final AddressableLEDBuffer m_ledBuffer;

  private CircularLEDBuffer m_frontLeft;
  private CircularLEDBuffer m_frontRight;
  private CircularLEDBuffer m_rearLeft;
  private CircularLEDBuffer m_rearRight;

  private CircularLEDBuffer[] swerveLEDs;

  private DriveSubsystem m_drive;

  

  public SwerveLEDSubsystem(DriveSubsystem drive) {

    m_drive = drive;
    // Blue by default
    m_isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;

    
    m_ledStrip = new AddressableLED(LEDConstants.kLEDStripPort);
    m_ledBuffer = new AddressableLEDBuffer(m_length);

    AddressableLEDBufferView frontLeftView = m_ledBuffer.createView(0, m_length / 4 - 1);
    m_frontLeft = new CircularLEDBuffer(frontLeftView, LEDConstants.kFrontLeftLEDAngularOffset);

    AddressableLEDBufferView frontRightView = m_ledBuffer.createView(m_length / 4, m_length / 2 - 1);
    m_frontRight = new CircularLEDBuffer(frontRightView, LEDConstants.kFrontRightLEDAngularOffset);

    AddressableLEDBufferView rearRightView = m_ledBuffer.createView(m_length / 2, 3 * m_length / 4 - 1);
    m_rearRight = new CircularLEDBuffer(rearRightView, LEDConstants.kBackLeftLEDAngularOffset);

    AddressableLEDBufferView rearLeftView = m_ledBuffer.createView(3 * m_length / 4, m_length - 1);
    m_rearLeft = new CircularLEDBuffer(rearLeftView, LEDConstants.kBackRightLEDAngularOffset);

    swerveLEDs = new CircularLEDBuffer[] {m_frontLeft, m_frontRight, m_rearLeft, m_rearRight};
    
    m_ledStrip.setLength(m_ledBuffer.getLength());

    

    m_ledStrip.start();

    setFreedomPattern();
    //m_ledStrip.setData(m_ledBuffer);
    
  }

  @Override
  public void periodic() {
    setFreedomPattern();
    m_ledStrip.setData(m_ledBuffer); 
  }



  public void setFreedomPattern() {
    for (int i = 0; i < swerveLEDs.length; i++) {
      swerveLEDs[i].setFreedomPattern();
    }
  }


 

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve LEDs");

  }
}