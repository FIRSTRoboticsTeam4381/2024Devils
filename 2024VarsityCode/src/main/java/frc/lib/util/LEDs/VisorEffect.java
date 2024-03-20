package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;

/** Simple effect with a blank background and a bar ("visor") of LEDs moving back and forth */
public class VisorEffect extends LightingEffect{

    /* ATTRIBUTES */

    private Color visorColor;
    private int visorLocation;
    private int visorSize;
    private double pixPerMs;
    private double msPerPix;

    private int movementModifier = -1;
    private double timer;


    /* CONSTRUCTORS */

    /**
     * Create a new VisorEffect. A simple collection of LEDs that bounce back and forth accross an LED strip
     * @param startLED The beginning of the defined LED range for the effect. Determines how many LEDs the effect occupies
     * @param lastLED The final LED of the defined range for the effect. Determines how many LEDs the effect occupies
     * @param visorColor
     * @param visorSize
     * @param speed Speed that the visor moves. Measured in LED pixels per second. 
     * @param startLocation Specify a starting location for the visor if you want it to start somewhere other than 0
     */
    public VisorEffect(int startLED, int lastLED, Color visorColor, int visorSize, double speed, int startLocation, Type type){
        super(startLED, lastLED, type);
        this.visorColor = visorColor;
        this.visorSize = visorSize;

        pixPerMs = speed/100.0;
        msPerPix = 1.0/pixPerMs;

        visorLocation = startLocation;
        this.timer = 0;
    }

    /**
     * Create a new VisorEffect. A simple collection of LEDs that bounce back and forth accross an LED strip
     * @param bufferLength The number of pixels for the effect to occupy. Assumes starting at index 0. To specify a start location, use the other constructor.
     * @param visorColor
     * @param visorSize
     * @param speed Speed that the visor moves. Measured in LED pixels per second. 
     * @param startLocation Specify a starting location for the visor if you want it to start somewhere other than 0
     */
    public VisorEffect(int bufferLength, Color visorColor, int visorSize, double speed, int startLocation, Type type){
        this(0, bufferLength-1, visorColor, visorSize, speed, 0, type);
    }

    /**
     * Create a new VisorEffect. A simple collection of LEDs that bounce back and forth accross an LED strip. Assumes that the visor starts at position = 0
     * @param startLED The beginning of the defined LED range for the effect. Determines how many LEDs the effect occupies
     * @param lastLED The final LED of the defined range for the effect. Determines how many LEDs the effect occupies
     * @param visorColor
     * @param visorSize
     * @param speed Speed that the visor moves. Measured in LED pixels per second. 
     */
    public VisorEffect(int startLED, int lastLED, Color visorColor, int visorSize, double speed, Type type){
        this(startLED, lastLED, visorColor, visorSize, speed, 0, type);
    }
    
    /**
     * Create a new VisorEffect. A simple collection of LEDs that bounce back and forth accross an LED strip. Assumes that the visor starts at position = 0
     * @param bufferLength The number of pixels for the effect to occupy. Assumes starting at index 0. To specify a start location, use the other constructor.
     * @param visorColor
     * @param visorSize
     * @param speed Speed that the visor moves. Measured in LED pixels per second. 
     */
    public VisorEffect(int bufferLength, Color visorColor, int visorSize, double speed, Type type){
        this(bufferLength, visorColor, visorSize, speed, 0, type);
    }


    /* METHODS */

    @Override
    public Color[] updatePixels(){
        // Creates a new color array and ticks the timer up by 20 ms (because this is called every 20 ms when active)
        Color[] pixels = new Color[bufferLength];
        timer += 2.0;

        // If enough time has passed (defined by speed), move the visor location forward by 1
        if(timer >= msPerPix){
            visorLocation += movementModifier;
            timer = 0;
        }

        // Set all pixels to off except for the visor pixels, which are set to the provided color
        for(int i = 0; i < bufferLength; i++){
            if (i > visorLocation-visorSize && i < visorLocation+visorSize) pixels[i] = visorColor;
            else pixels[i] = new Color(0,0,0);
        }

        // If the visor has reached one of the ends of the LED strip, flip the direction
        if(visorLocation <= 0){
            movementModifier = 1;
        }
        else if(visorLocation >= bufferLength-1){
            movementModifier = -1;
        }

        return pixels;
    }
}
