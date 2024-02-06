// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDs.LightingEffect;
import frc.lib.util.LEDs.SolidColorEffect;
import frc.robot.Constants;

/* Note for self. The example usage of AddressableLED and AddressableLEDBuffer had
 * nor problem with repeatedly using led.set(buffer) every periodic call. This means
 * that a new buffer can be created every periodic and set to the LEDs
 */
public class LEDs extends SubsystemBase {
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  private int ledLength = 0;

  private LightingEffect effect;
  
  /** Creates a new LEDs. */
  public LEDs(int ledLength) {
    effect = new SolidColorEffect(ledLength, new int[] {0,0,0});
    this.ledLength = ledLength;
    led = new AddressableLED(Constants.ledPWM);
    led.setLength(ledLength);
    ledBuffer = effect.updateBuffer();
    led.setData(ledBuffer);
    led.start();
  }

  public void setEffect(LightingEffect effect){
    this.effect = effect;
  }

  @Override
  public void periodic() {
    led.setData(effect.updateBuffer());
    // This method will be called once per scheduler run
  }
}
