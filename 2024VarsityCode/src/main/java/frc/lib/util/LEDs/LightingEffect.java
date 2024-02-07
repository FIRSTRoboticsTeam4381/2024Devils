package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

/** Abstract class for defining dynamic lighting effects */
public abstract class LightingEffect {

    /* ATTRIBUTES */

    protected int bufferLength;
    protected int firstLED;
    protected int lastLED;


    /* CONSTRUCTORS */

    public LightingEffect(int firstLED, int lastLED){
        this.firstLED = firstLED;
        this.lastLED = lastLED;
        this.bufferLength = lastLED-firstLED+1;
    }


    /* METHODS */

    public int getFirstLED(){
        return firstLED;
    }
    
    public int getLastLED(){
        return lastLED;
    }

    public int getLength(){
        return bufferLength;
    }


    /* ABSTRACT METHODS */
    
    /**
     * Get the color array of the effect, with each LED pixel represented by one color. Is the size specified by lastLED-firstLED+1 as
     * provided in the constructor. Also ticks forward any dynamic effects that this lighting effect does.
     */
    public abstract Color[] updatePixels();
}
