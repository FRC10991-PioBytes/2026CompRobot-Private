// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.wpilibj.util.Color;


public class CircularLEDBuffer {
  // Instantiates the motors and encoders
  private int m_length;
  private Rotation2d m_angularOffset;
  private Rotation2d m_angularSetpoint;

  private AddressableLEDBufferView m_bufferView;

  /**
   * Constructs a Circular Buffer for LEDs
   */
  public CircularLEDBuffer(AddressableLEDBufferView bufferView, Rotation2d angularOffset) {
    m_bufferView = bufferView;

    m_length = m_bufferView.getLength();
    m_angularOffset = angularOffset;
  }

  public void setLookingForTargetPattern() {
    LEDPattern whitePattern = LEDPattern.solid(Color.kWhite);
    whitePattern.applyTo(m_bufferView);
  }

  public void setTargetFoundPattern() {
    LEDPattern whitePattern = LEDPattern.solid(Color.kGreen);
    whitePattern.applyTo(m_bufferView);
  }

  public void setLoadingPattern() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      LEDPattern loadingPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlack, Color.kRed);
      loadingPattern.scrollAtRelativeSpeed(Frequency.ofBaseUnits(1, Units.Hertz));
      loadingPattern.applyTo(m_bufferView);
    }
    else {
      LEDPattern loadingPattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlack, Color.kBlue);
      loadingPattern.scrollAtRelativeSpeed(Frequency.ofBaseUnits(1, Units.Hertz));
      loadingPattern.applyTo(m_bufferView);
    }
  }

  public void setOff() {
    LEDPattern blackPattern = LEDPattern.solid(Color.kBlack);
    blackPattern.applyTo(m_bufferView);
  }

  // Sets the Azimuth Pattern given a wheel direction
  public void setAzimuthPattern(Rotation2d wheelDirection) {
    // Get the LED-oriented direction
    Rotation2d LEDDirection = applyAngularOffset(wheelDirection);
    m_angularSetpoint = LEDDirection;
    // Get the closest LEDIndex
    int closestLEDIndex = convertToLEDIndex(LEDDirection);
    // Get the oppositve LEDIndex
    int oppositeLEDIndex = getOppositeLEDIndex(closestLEDIndex);

    // Set the Azimuth LED Pattern
    // Red by default, blue if alliance is blue
    LEDPattern whitePattern = LEDPattern.solid(Color.kWhite);
    whitePattern.applyTo(m_bufferView);

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      this.setColor(closestLEDIndex, Color.kBlue);
      this.setColor(oppositeLEDIndex, Color.kBlue);
    }
    else {
      this.setColor(closestLEDIndex, Color.kRed);
      this.setColor(oppositeLEDIndex, Color.kRed);
    }

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

  // Returns closest LEDIndex with range of (0, length]
  public int convertToLEDIndex(Rotation2d angle) {
    double LEDegree = (angle.getDegrees() / 360) * m_length;
    // Change range from (-length / 2, length / 2] to (0, length]
    if (LEDegree < 0) {
      LEDegree += m_length;
    }

    // Round the degree
    int LEDIndex = ((int) (LEDegree + 0.5));
    // Make index start at 0
    LEDIndex--;

    return LEDIndex;
  }

  // Sets the color of the LED
  public void setColor(int index, Color color) {
    m_bufferView.setRGB(index, (int) (color.red * 255 + 0.5), (int) (color.green * 255 + 0.5), (int) (color.blue * 255 + 0.5));
  }

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