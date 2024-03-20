// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDs.ConditionalSolidColor;
import frc.lib.util.LEDs.LEDWrapper;
import frc.lib.util.LEDs.LightingEffect;
import frc.lib.util.LEDs.SolidColorEffect;

/* Note for self. The example usage of AddressableLED and AddressableLEDBuffer had
 * not problem with repeatedly using led.set(buffer) every periodic call. This means
 * that a new buffer can be created every periodic and set to the LEDs
 */
public class LEDs extends SubsystemBase {

    /* ATTRIBUTES */

    // A map of LED strips and their respective identifier keys
    private Map<String, LEDWrapper> ledStrips = new HashMap<String, LEDWrapper>();
    

    /* CONSTRUCTORS */

    /** Creates a new LEDs.
     * @param ledLength The number of LEDs on the LED strip
     * @param port The pwm port the strip is plugged into
    */
    public LEDs() {
        // Create list of led strips and respective keys for identifying
        // TODO get ports and lengths
        ledStrips.put("shooter-right", new LEDWrapper(0, 100));
        ledStrips.put("shooter-left", new LEDWrapper(0, 100));
        ledStrips.put("climb-right", new LEDWrapper(0, 100));
        ledStrips.put("climb-left", new LEDWrapper(0, 100));

        // Create a list of effects associated with each LED strip
        for(String s : ledStrips.keySet()){
            AddressableLED aled = ledStrips.get(s);
            activeEffects.put(aled, new ArrayList<LightingEffect>());
            clear(aled);
        }


        activeEffects = new ArrayList<LightingEffect>();
        activeEffects.add(new SolidColorEffect(ledLength, new Color(0,0,0)));

        this.ledLength = ledLength;
        led = new AddressableLED(port);
        led.setLength(ledLength);

        led.setData(updateEffects());
        led.start();
    }


    /* METHODS */
    private void clear(AddressableLED aled){
        activeEffects.get(aled).clear();
        activeEffects.get(aled).add(new SolidColorEffect(aled.))
    }
}
