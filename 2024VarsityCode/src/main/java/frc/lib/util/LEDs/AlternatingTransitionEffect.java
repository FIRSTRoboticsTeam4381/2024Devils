package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;

/** Alternating color wave effect with a smooth transition */
public class AlternatingTransitionEffect extends LightingEffect{

    /* ATTRIBUTES */

    private ArrayList<Color> stepList;
    private int index = 0;
    private double pixPerMs;
    private double msPerPix;
    private double timer;


    /* CONSTRUCTORS */

    /**
     * Create a new AlternatingTransitionEffect. Acts like an even colorwave between two colors.
     * @param startLED The beginning of the defined LED range for the effect. Determines how many LEDs the effect occupies
     * @param lastLED The final LED of the defined range for the effect. Determines how many LEDs the effect occupies
     * @param color1 
     * @param color2
     * @param waveLength The length of each resulting wave. Think of this like a sine wave, this is the period length. Measured in LED pixels
     * @param speed The speed of the wave effect. Measured in LED pixels per second. Expected range 1-50, values outside of that will cause unknown behavior
     */
    public AlternatingTransitionEffect(int startLED, int lastLED, Color color1, Color color2, int waveLength, double speed){
        super(startLED, lastLED);
        stepList = new ArrayList<Color>();
        this.timer = 0;
        pixPerMs = speed/100.0;
        msPerPix = 1.0/pixPerMs;

        Color[] foreSteps = calcSteps(color1, color2, waveLength/2);
        Color[] backSteps = calcSteps(color2, color1, waveLength/2);

        for(int i = 0; i < foreSteps.length; i++){
            stepList.add(foreSteps[i]);
        }
        for(int i = 0; i < backSteps.length; i++){
            stepList.add(backSteps[i]);
        }
    }
    
    /**
     * Create a new AlternatingTransitionEffect. Acts like an even colorwave between two colors.
     * @param bufferLength The number of pixels for the effect to occupy. Assumes starting at index 0. To specify a start location, use the other constructor.
     * @param color1 
     * @param color2
     * @param waveLength The length of each resulting wave. Think of this like a sine wave, this is the period length. Measured in LED pixels
     * @param speed The speed of the wave effect. Measured in LED pixels per second. Expected range 1-50, values outside of that will cause unknown behavior
     */
    public AlternatingTransitionEffect(int bufferLength, Color color1, Color color2, int waveLength, double speed){
        this(0, bufferLength-1, color1, color2, waveLength, speed);
    }


    /* METHODS */

    /**
     * Calculate all of the color steps needed to evenly transition between two given colors. Uses a simple linear algorithm
     * to adjust red, green, and blue channels simultaneously to attempt a smooth transition by finding the slope between two
     * color points
     * @param color1 The color to start at. This is point #1 on the graph
     * @param color2 The target color. This is point #2
     * @param numSteps The number of steps to achieve the transition in. More steps = smaller steps = longer but smoother transition
     * @return
     */
    private Color[] calcSteps(Color color1, Color color2, int numSteps){
        Color[] steps = new Color[numSteps];

        // Calculate the theoretical slopes between two points where y is color value and x is number of steps (simple rise over run)
        double rSlope = (color2.red-color1.red)/(double)numSteps;
        double gSlope = (color2.green-color1.green)/(double)numSteps;
        double bSlope = (color2.blue-color1.blue)/(double)numSteps;

        // Using the calculated slopes, calculate each necessary step by multiplying slope and step number together
        for(int i = 0; i < numSteps; i++){
            steps[i] = new Color(color1.red+(rSlope*i), color1.green+(gSlope*i), color1.blue+(bSlope*i));
        }
        return steps;
    }

    @Override
    public Color[] updatePixels(){
        // Create a color array and tick the timer up by 20 ms (because this will be called every 20 ms if active)
        Color[] pixels = new Color[bufferLength];
        timer += 2.0;

        // If enough time has passed (as specified by speed), move the position forward by 1 pixel
        if(timer >= msPerPix){
            index+=1;
            timer = 0;
        }

        // Using the current location (stored in index) create the resulting wave pattern from the steplist that has already been calculated
        for(int i = 0; i < bufferLength; i++){
            Color color = stepList.get((i+index)%stepList.size());
            pixels[i] = color;
        }

        // Reset the index back to a lower number because it will get stupid big
        if(index>1000000) index%=stepList.size();

        return pixels;
    }
}
