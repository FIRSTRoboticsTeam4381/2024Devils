package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class SolidColorEffect extends LightingEffect{
    private Color color;

    public SolidColorEffect(int startLED, int lastLED, Color color){
        super(startLED, lastLED);
        this.color = color;
    }
    public SolidColorEffect(int bufferLength, Color color){
        this(0, bufferLength-1, color);
    }

    @Override
    public Color[] updatePixels() {
        return updatePixelsAtPercent(1);
    }

    public Color[] updatePixelsAtPercent(double p){
        Color[] pixels = new Color[bufferLength];
        for(int i = 0; i < bufferLength; i++){
            pixels[i] = new Color(color.red*p, color.green*p, color.blue*p);
        }
        return pixels;
    }
}
