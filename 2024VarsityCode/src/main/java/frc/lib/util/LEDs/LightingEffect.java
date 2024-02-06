package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public abstract class LightingEffect {
    protected int bufferLength;
    public LightingEffect(int bufferLength){
        this.bufferLength = bufferLength;
    }
    public abstract AddressableLEDBuffer updateBuffer();
}
