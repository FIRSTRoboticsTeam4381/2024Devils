// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDs.LEDZone;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private AddressableLED leds;
  private int totalLength;
  private LEDZone[] lightingZones;

  /** Creates a new LEDs. */
  public LEDs() {
    leds = new AddressableLED(Constants.ledPort);
    lightingZones = new LEDZone[] {
      new LEDZone("shooter-right", 0, 100),
      new LEDZone("shooter-left", 100, 100),
      new LEDZone("climb-right", 200, 100),
      new LEDZone("climb-left", 300, 100)
    };

    {
      int sum = 0;
      for(LEDZone z : lightingZones){
        sum+=z.getLength();
      }
      totalLength = sum;
    }
    leds.setLength(totalLength);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color[] pixels = new Color[totalLength];
    for(LEDZone z : lightingZones){
      z.update();
      Color[] zonePixels = z.getPixels();
      for(int i = 0; i < z.getLength(); i++){
        pixels[i+z.getLocation()] = zonePixels[i];
      }
    }

    AddressableLEDBuffer buffer = new AddressableLEDBuffer(totalLength);
    for(int i = 0; i < totalLength; i++){
      buffer.setRGB(i, (int)pixels[i].red, (int)pixels[i].green, (int)pixels[i].blue);
    }
    leds.setData(buffer);
  }
}
