package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class SolidColorEffect extends LightingEffect{
    private int[] color;

    public SolidColorEffect(int bufferLength, int[] color){
        super(bufferLength);
        this.color = color;
    }

    @Override
    public AddressableLEDBuffer updateBuffer() {
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(bufferLength);
        for(int i = 0; i < bufferLength; i++){
            buffer.setRGB(i, color[0], color[1], color[2]);
        }
        return buffer;
    }
}
