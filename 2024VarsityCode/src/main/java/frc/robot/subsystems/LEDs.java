// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDs.Colors;
import frc.lib.util.LEDs.LightingEffect;
import frc.lib.util.LEDs.SolidColorEffect;
import frc.robot.Constants;

/* Note for self. The example usage of AddressableLED and AddressableLEDBuffer had
 * not problem with repeatedly using led.set(buffer) every periodic call. This means
 * that a new buffer can be created every periodic and set to the LEDs
 */
public class LEDs extends SubsystemBase {
  private AddressableLED led;
  private int ledLength = 0;

  private ArrayList<LightingEffect> activeEffects;
  
  /** Creates a new LEDs. */
  public LEDs(int ledLength, int port) {
    activeEffects = new ArrayList<LightingEffect>();
    activeEffects.add(new SolidColorEffect(ledLength, Colors.OFF));

    this.ledLength = ledLength;
    led = new AddressableLED(port);
    led.setLength(ledLength);

    led.setData(updateEffects());
    led.start();
  }

  public AddressableLEDBuffer updateEffects(){
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(ledLength);

    for(int i = 0; i < buffer.getLength(); i++){
      buffer.setRGB(i, 0, 0, 0);
    }
    for(LightingEffect e : activeEffects){
      AddressableLEDBuffer b = e.updateBuffer();
      for(int i = 0; i < ledLength; i++){
        if(b.getLED(i).red!=0||b.getLED(i).green!=0||b.getLED(i).blue!=0) // If the LED has an active color effect
          buffer.setRGB(i, b.getLED8Bit(i).red, b.getLED8Bit(i).green, b.getLED8Bit(i).blue);
      }
    }
    return buffer;
  }
  /**
   * Add an effect to the stack. Sends the effect to the bottom of the load order by default
   * (lower in load order overwrites higher in load order)
   * @param effect
   */
  public void addEffect(LightingEffect... effects){
    for(int i = 0; i < effects.length; i++){
      activeEffects.add(effects[i]);
    }
  }
  /**
   * Add an effect to the stack. Specify the position in the load order to send the effect to
   * (lower in load order (higher index) overwrites effects that load first (lower index))
   * @param effect
   * @param position
   */
  public void addEffect(int index, LightingEffect... effects){
    for(int i = 0; i < effects.length; i++){
      activeEffects.add(index+i, effects[i]);
    }
  }
  public void removeEffect(int index){
    activeEffects.remove(index);
  }
  public void clearEffects(){
    activeEffects.clear();
    activeEffects.add(new SolidColorEffect(ledLength, Colors.OFF));
  }
  public void setEffect(LightingEffect... effects){
    clearEffects();
    addEffect(effects);
  }

  @Override
  public void periodic() {
    AddressableLEDBuffer buffer = updateEffects();
    led.setData(buffer);
    // This method will be called once per scheduler run
  }
}
