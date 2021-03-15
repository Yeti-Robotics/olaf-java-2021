// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;
  public LEDSubsystem() {
    ledStrip = new AddressableLED(LEDConstants.ADDRESSABLE_LED);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);
    ledStrip.setLength(ledBuffer.getLength());
  }

  public void setHSV(int i, int hue, int saturation, int value){
      ledBuffer.setHSV(i, hue, saturation, value);
  }
  
  public void setRGB(int i, int red, int green, int cyan){
   ledBuffer.setRGB(i, red, green, cyan);
  }

  public int getBufferLength(){
    return ledBuffer.getLength();
  }

  public void sendData(){
    ledStrip.setData(ledBuffer);
  }

}
