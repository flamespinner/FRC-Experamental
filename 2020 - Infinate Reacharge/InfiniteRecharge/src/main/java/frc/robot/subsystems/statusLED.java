package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class statusLED extends SubsystemBase {
  /**
   * Creates a new Lighting Subsystem.
   */
  private static AddressableLED m_led;
  private static AddressableLEDBuffer m_ledBuffer;
  private static int m_rainbowFirstPixelHue;

  public statusLED() {
    m_led = new AddressableLED(1);
    m_ledBuffer = new AddressableLEDBuffer(120);
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }


  @Override
  public void periodic() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void LEDtargetFound() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 57, 255, 20);
    }
    m_led.setData(m_ledBuffer);
  }

  public void LEDshooterAtVelocity() {
    //make the leds neon green
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 57, 255, 20);
    }
    m_led.setData(m_ledBuffer);
  }

  public void LEDBlue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 255);
    }
    m_led.setData(m_ledBuffer); 
  }

  public void LEDRainbow(){
    //--- make a rainbow pattern on LEDs ---//
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
        m_ledBuffer.setHSV(i, hue, 255, 128);
      }
      m_rainbowFirstPixelHue += 3;
      m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }

}