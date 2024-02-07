package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayList;

public class AlternatingTransitionEffect extends LightingEffect{
    private ArrayList<Color> stepList;

    private int index = 0;
    private int speed;

    public AlternatingTransitionEffect(int startLED, int lastLED, Color color1, Color color2, int waveLength, int speed){
        super(startLED, lastLED);
        stepList = new ArrayList<Color>();
        this.speed = speed;

        Color[] foreSteps = calcSteps(color1, color2, waveLength/2);
        Color[] backSteps = calcSteps(color2, color1, waveLength/2);

        for(int i = 0; i < foreSteps.length; i++){
            stepList.add(foreSteps[i]);
        }
        for(int i = 0; i < backSteps.length; i++){
            stepList.add(backSteps[i]);
        }
    }
    public AlternatingTransitionEffect(int bufferLength, Color color1, Color color2, int waveLength, int speed){
        this(0, bufferLength-1, color1, color2, waveLength, speed);
    }

    private Color[] calcSteps(Color color1, Color color2, int numSteps){
        Color[] steps = new Color[numSteps];

        double rSlope = (color2.red-color1.red)/(double)numSteps;
        double gSlope = (color2.green-color1.green)/(double)numSteps;
        double bSlope = (color2.blue-color1.blue)/(double)numSteps;

        for(int i = 0; i < numSteps; i++){
            steps[i] = new Color(color1.red+(rSlope*i), color1.green+(gSlope*i), color1.blue+(bSlope*i));
        }
        return steps;
    }

    @Override
    public Color[] updatePixels(){
        Color[] pixels = new Color[bufferLength];

        for(int i = 0; i < bufferLength; i++){
            Color color = stepList.get((i+index)%stepList.size());
            pixels[i] = color;
        }
        index+=speed;
        if(index>1000000) index%=stepList.size();
        return pixels;
    }
}
