package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;

/** Set all LEDs in a range to a solid color */
public class SolidColorEffect extends LightingEffect{

    /* ATTRIBUTES */

    protected Color color;


    /* CONSTRUCTORS */

    /**
     * Creates a new SolidColorEffect. Sets all LEDs in the defined range to the provided color
     * @param startLED The beginning of the defined LED range for the effect. Determines how many LEDs the effect occupies
     * @param lastLED The final LED of the defined range for the effect. Determines how many LEDs the effect occupies
     * @param color The color to set the LEDs to
     */
    public SolidColorEffect(int startLED, int lastLED, Color color, Type type){
        super(startLED, lastLED, type);
        this.color = color;
    }

    /**
     * Creates a new SolidColorEffect. Sets all LEDs in the defined range to the provided color
     * @param bufferLength The number of pixels for the effect to occupy. Assumes starting at index 0. To specify a start location, use the other constructor.
     * @param color The color to set the LEDs to
     */
    public SolidColorEffect(int bufferLength, Color color, Type type){
        this(0, bufferLength-1, color, type);
    }


    /* METHODS */
    
    @Override
    public Color[] updatePixels() {
        return updatePixelsAtPercent(1);
    }

    /**
     * Does the same thing as the standard updatePixels method but allows the variability of LED brightness
     */
    public Color[] updatePixelsAtPercent(double p){
        
        // Create a new color array and set all of the pixels to the provided color multiplied by percent brightness
        Color[] pixels = new Color[bufferLength];
        for(int i = 0; i < bufferLength; i++){
            pixels[i] = new Color(color.red*p, color.green*p, color.blue*p);
        }
        return pixels;
    }
}
