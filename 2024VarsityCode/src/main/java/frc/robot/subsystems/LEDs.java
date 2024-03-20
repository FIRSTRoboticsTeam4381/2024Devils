// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDs.LEDWrapper;
import frc.lib.util.LEDs.LightingEffect;

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

    }


    /* COMMANDS */
    public Command addEffect(String key, LightingEffect e){
        return new InstantCommand(()->ledStrips.get(key).addEffect(e));
    }
    public Command setEffects(String key, LightingEffect... e){
        return new InstantCommand(()->ledStrips.get(key).setEffects(e));
    }
    public Command clearEffects(String key){
        return new InstantCommand(()->ledStrips.get(key).clearEffects());
    }
    public Command addEffectToAll(LightingEffect e){
        return new ParallelCommandGroup(addEffect("shooter-right",e),
                                        addEffect("shooter-left",e),
                                        addEffect("climb-right",e),
                                        addEffect("climb-left",e)
                                        );
    }
    public Command setEffectsToAll(LightingEffect... e){
        return new ParallelCommandGroup(setEffects("shooter-right",e),
                                        setEffects("shooter-left",e),
                                        setEffects("climb-right",e),
                                        setEffects("climb-left",e)
                                        );
    }
    public Command clearEffectsFromAll(){
        return new ParallelCommandGroup(clearEffects("shooter-right"),
                                        clearEffects("shooter-left"),
                                        clearEffects("climb-right"),
                                        clearEffects("climb-left"));
    }
    
    @Override
    public void periodic(){
        for(String s : ledStrips.keySet()){
            ledStrips.get(s).periodic();
        }
    }
}
