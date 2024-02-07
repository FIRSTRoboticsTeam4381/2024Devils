// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDs.LightingEffect;
import frc.lib.util.LEDs.SolidColorEffect;
import frc.robot.Constants;

/* Note for self. The example usage of AddressableLED and AddressableLEDBuffer had
 * not problem with repeatedly using led.set(buffer) every periodic call. This means
 * that a new buffer can be created every periodic and set to the LEDs
 */
public class LEDs extends SubsystemBase {

    /* ATTRIBUTES */
    
    private AddressableLED led;
    private int ledLength = 0;

    private ArrayList<LightingEffect> activeEffects;
    

    /* CONSTRUCTORS */

    /** Creates a new LEDs.
     * @param ledLength The number of LEDs on the LED strip
     * @param port The pwm port the strip is plugged into
    */
    public LEDs(int ledLength, int port) {
        // Create the effect stack and set the lights to off
        activeEffects = new ArrayList<LightingEffect>();
        activeEffects.add(new SolidColorEffect(ledLength, new Color(0,0,0)));

        this.ledLength = ledLength;
        led = new AddressableLED(port);
        led.setLength(ledLength);

        led.setData(updateEffects());
        led.start();
    }


    /* METHODS */

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

    /**
     * Remove an effect at the specified index
     * @param index The index of the effect to remove from the stack
     */
    public void removeEffect(int index){
        activeEffects.remove(index);
    }

    /**
     * Clears all of the effects from the stack
     */
    public void clearEffects(){
        activeEffects.clear();
    }

    /**
     * Clears all current effects from the stack and adds the provided effects, activating them instantly
     * @param effects The effects to add to the stack
     */
    public void setEffect(LightingEffect... effects){
        clearEffects();
        addEffect(effects);
    }

    /**
     * A multi-step method that is crucial to the functionality of this effect framework. Creates a new LEDBuffer that will be supplied back
     * to the calling function. Sets all the LEDs to off, then polls each active effect to get their current status and layers them on
     * top of each other. Also ensures that effects that have specific positions will only be applied to those positions.
     * @return The AddressableLEDBuffer that contains every active effect layered on top of each other
     */
    public AddressableLEDBuffer updateEffects(){
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(ledLength);
        for(int i = 0; i < ledLength; i++){
            buffer.setRGB(i,0,0,0);
        }
        for(LightingEffect e : activeEffects){
            Color[] effectPixels = e.updatePixels();
            int location = e.getFirstLED();
            for(int i = location; i < effectPixels.length; i++){
                if(effectPixels[i-location].red!=0||effectPixels[i-location].green!=0||effectPixels[i-location].blue!=0)
                    buffer.setRGB(i, (int)(effectPixels[i].red*255), (int)(effectPixels[i].green*255), (int)(effectPixels[i].blue*255));
            }
        }

        return buffer;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        led.setData(updateEffects());
    }
}
