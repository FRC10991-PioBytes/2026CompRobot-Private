// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDReader;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;


public class CircularLEDBuffer {
  // Instantiates the motors and encoders
  private final int m_length;
  private final Rotation2d m_angularOffset;
  private Rotation2d m_angularSetpoint;

  private AddressableLEDBufferView m_bufferView;

  private static final LEDPattern kWhitePattern = LEDPattern.solid(new Color(25, 25, 25));
  private static final LEDPattern kGreenPattern = LEDPattern.solid(Color.kGreen);
  private static final LEDPattern kBlackPattern = LEDPattern.solid(Color.kBlack);

  private static final LEDPattern kMask = LEDPattern.steps(Map.of(0, Color.kBlack, 0.25, Color.kWhite));
  private static final LEDPattern kRedLoadingPattern =
    LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, Color.kRed)
      //.mask(kMask)
        .scrollAtRelativeSpeed(Frequency.ofBaseUnits(1, Units.Hertz));

  private static final LEDPattern kBlueLoadingPattern =
    LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, Color.kBlue)
      //.mask(kMask)
        .scrollAtRelativeSpeed(Frequency.ofBaseUnits(1, Units.Hertz));

  /**
   * Constructs a Circular Buffer for LEDs
   */
  public CircularLEDBuffer(AddressableLEDBufferView bufferView, Rotation2d angularOffset) {
    m_bufferView = bufferView.reversed();

    m_length = m_bufferView.getLength();
    m_angularOffset = angularOffset;
  }

  public void setLookingForTargetPattern() {
    kWhitePattern.applyTo(m_bufferView);
  }

  public void setTargetFoundPattern() {
    kGreenPattern.applyTo(m_bufferView);
  }

  public void setLoadingPattern(boolean isRed) {
    if (isRed) {
      kRedLoadingPattern.applyTo(m_bufferView);
    }
    else {
      kBlueLoadingPattern.applyTo(m_bufferView);
    }
  }

  public void setOff() {
    kBlackPattern.applyTo(m_bufferView);
  }

  // Sets the Azimuth Pattern given a wheel direction
  public void setAzimuthPattern(Rotation2d wheelDirection, boolean isRed) {
    // Get the LED-oriented direction
    Rotation2d LEDDirection = applyAngularOffset(wheelDirection);
    m_angularSetpoint = LEDDirection;
    // Get the closest LEDIndex
    int closestLEDIndex = convertToLEDIndex(LEDDirection);
    // Get the opposite LEDIndex
    int oppositeLEDIndex = getOppositeLEDIndex(closestLEDIndex);

    // Set the Azimuth LED Pattern
    // Red by default, blue if alliance is blue
    kBlackPattern.applyTo(m_bufferView);

    int[] indexGroups = new int[6];
    indexGroups[0] = (closestLEDIndex == 0) ? m_length - 1 : closestLEDIndex - 1;
    indexGroups[1] = closestLEDIndex;
    indexGroups[2] = (closestLEDIndex == m_length - 1) ? 0 : closestLEDIndex + 1;
    indexGroups[3] = (oppositeLEDIndex == 0) ? m_length - 1 : oppositeLEDIndex - 1;
    indexGroups[4] = oppositeLEDIndex;
    indexGroups[5] = (oppositeLEDIndex == m_length - 1) ? 0 : oppositeLEDIndex + 1;
    if (isRed) {
      for (int i = 0; i < 6; i++) {
        m_bufferView.setLED(indexGroups[i], Color.kRed);
      }
    }
    else {
      for (int i = 0; i < 6; i++) {
        m_bufferView.setLED(indexGroups[i], Color.kBlue);
      }
    }

    /*
    if (isRed) {
      m_bufferView.setLED(closestLEDIndex, Color.kRed);
      m_bufferView.setLED(oppositeLEDIndex, Color.kRed);
    }
    else {
      m_bufferView.setLED(closestLEDIndex, Color.kBlue);
      m_bufferView.setLED(oppositeLEDIndex, Color.kBlue);
    }
    */

    /*
    for (int i = 0; i < m_length; i++) {
      if (i == closestLEDIndex || i == oppositeLEDIndex)
      {
        // Red by default, blue if alliance is blue
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
          this.setColor(i, Color.kBlue);
        }
        else {
          this.setColor(i, Color.kRed);
        }
      }
      // Others are white
      else {
        this.setColor(i, Color.kWhite);
      }
    }
    */

    
  }
  
  /*
    Helper Methods
  */

  // Returns the LEDIndex (int) opposite a given LEDIndex
  public int getOppositeLEDIndex(int LEDIndex) {
    return (int) ((double) m_length / 2 + LEDIndex) % m_length;
  }

  // Returns a Rotation2d with an angular offset applied
  public Rotation2d applyAngularOffset(Rotation2d angle) {
    return angle.minus(m_angularOffset);
  }

  // Returns closest LEDIndex with range of [0, length)
  public int convertToLEDIndex(Rotation2d angle) {
    double newAngle = (angle.getRotations() % 1);
    if (newAngle < 0) {
      newAngle += 1;
    }

    double LEDegree = newAngle * m_length;
    return (int) (LEDegree + 0.5) % m_length;

    
  }

  /*
  // Sets the color of the LED
  public void setColor(int index, Color color) {
    m_bufferView.setLED(index, color);
    //m_bufferView.setRGB(index, (int) (color.red * 255 + 0.5), (int) (color.green * 255 + 0.5), (int) (color.blue * 255 + 0.5));
  }
  */

  /*
    Getters
  */

  public Rotation2d getAngularSetpoint() {
    return m_angularSetpoint;
  }

  /*
    Set Azimuth LED:
      Input: LED (int from 0 to buffer length)
      Output: sets all rgb to white except for the two in that direction

    Convert to LEDegrees:
      Input: Rotation2d
      Output: closest LED to that rotation
    
    Apply offset:
      Input: Rotation2d
      Output: new Rotation2d offset by the angular Offset

    Set Azimuth LEDPattern (input Rotation2d)
      1. Apply offset
      2. Find the closest LED
      3. Set Azimuth LED
  */
}