package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public abstract class LightingEffect {
    protected int bufferLength;
    protected int firstLED;
    protected int lastLED;
    public LightingEffect(int firstLED, int lastLED){
        this.firstLED = firstLED;
        this.lastLED = lastLED;
        this.bufferLength = lastLED-firstLED+1;
    }
    public int getFirstLED(){
        return firstLED;
    }
    public int getLastLED(){
        return lastLED;
    }
    public int getLength(){
        return bufferLength;
    }
    public abstract Color[] updatePixels();
}
