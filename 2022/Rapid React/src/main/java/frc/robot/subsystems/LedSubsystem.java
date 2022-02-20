// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;


public class LedSubsystem extends SubsystemBase {
  //makes variables needed
  private static AddressableLED m_led = new AddressableLED(LedConstants.LED_Port);
  private static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LedConstants.LED_Length);
  private static int m_rainbowFirstPixelHue;
  
  
 
  /** Creates a new LedSubsystem. */
  public LedSubsystem() {
    m_led.setLength(m_ledBuffer.getLength());
  }

  public void rainbow() {
    // For every pixel
    for (var i = 0; i < LedConstants.LED_Length; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / LedConstants.LED_Length)) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  public void setRainbow() {
    rainbow();
    m_led.setData(m_ledBuffer);
    m_led.start();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
