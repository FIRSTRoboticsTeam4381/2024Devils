package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import java.util.ArrayList;

public class AlternatingTransitionEffect extends LightingEffect{
    private ArrayList<double[]> stepList;

    private int index = 0;
    private int speed;

    public AlternatingTransitionEffect(int bufferLength, int[] color1, int[] color2, int waveLength, int speed){
        super(bufferLength);
        stepList = new ArrayList<double[]>();
        this.speed = speed;

        double[][] foreSteps = calcSteps(color1, color2, waveLength/2);
        double[][] backSteps = calcSteps(color2, color1, waveLength/2);

        for(int i = 0; i < foreSteps.length; i++){
            stepList.add(foreSteps[i]);
        }
        for(int i = 0; i < backSteps.length; i++){
            stepList.add(backSteps[i]);
        }
    }

    private double[][] calcSteps(int[] color1, int[] color2, int numSteps){
        double[][] steps = new double[numSteps][3];
        double rSlope = (color2[0]-color1[0])/(double)steps.length;
        double gSlope = (color2[1]-color1[1])/(double)steps.length;
        double bSlope = (color2[2]-color1[2])/(double)steps.length;

        for(int i = 0; i < steps.length; i++){
            steps[i][0] = color1[0]+(rSlope*i);
            steps[i][1] = color1[1]+(gSlope*i);
            steps[i][2] = color1[2]+(bSlope*i);
        }
        return steps;
    }

    public AddressableLEDBuffer updateBuffer(){
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(bufferLength);

        for(int i = 0; i < bufferLength; i++){
            double[] color = stepList.get((i+index)%stepList.size());
            buffer.setRGB(i, (int)color[0], (int)color[1], (int)color[2]);
        }
        index+=speed;
        if(index>1000000) index%=stepList.size();
        return buffer;
    }
}
