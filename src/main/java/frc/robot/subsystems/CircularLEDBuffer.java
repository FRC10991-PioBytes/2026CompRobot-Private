// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
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

  // test comment
  private static final LEDPattern kRedLoadingPattern =
    LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, Color.kRed)
      //.mask(kMask)
        .scrollAtRelativeSpeed(Frequency.ofBaseUnits(1, Units.Hertz));

  private static final LEDPattern kBlueLoadingPattern =
    LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlack, Color.kBlue)
      //.mask(kMask)
        .scrollAtRelativeSpeed(Frequency.ofBaseUnits(1, Units.Hertz));

  private static final LEDPattern kFreedomPattern = 
    LEDPattern.steps(Map.of(0, Color.kRed, 0.33, Color.kWhite, 0.67, Color.kBlue))
    //LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kWhite, Color.kBlue)
      //.mask(kMask)
        .scrollAtRelativeSpeed(Frequency.ofBaseUnits(0.75, Units.Hertz));

  /**
   * Constructs a Circular Buffer for LEDs
   */
  public CircularLEDBuffer(AddressableLEDBufferView bufferView, Rotation2d angularOffset) {
    m_bufferView = bufferView.reversed();

    m_length = m_bufferView.getLength();
    m_angularOffset = angularOffset;
  }

  public void setFreedomPattern() {
    kFreedomPattern.applyTo(m_bufferView);
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