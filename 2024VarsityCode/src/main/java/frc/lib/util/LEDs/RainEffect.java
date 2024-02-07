package frc.lib.util.LEDs;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RainEffect extends LightingEffect{
    private int speed;

    private int maxVolume;
    private ArrayList<Integer> activeIndexes;
    private ArrayList<Integer> timers;
    private ArrayList<int[]> dropColors;

    private int[] color;

    public RainEffect(int bufferSize, int speed, int volume){
        super(bufferSize);
        this.speed = speed;
        this.maxVolume = volume;

        activeIndexes = new ArrayList<Integer>();
        timers = new ArrayList<Integer>();
        dropColors = new ArrayList<int[]>();
    }
    public RainEffect(int bufferSize, int[] rainColor, int speed, int volume){
        this(bufferSize, speed, volume);
        this.color = rainColor;
    }

    public AddressableLEDBuffer updateBuffer(){
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(bufferLength);
        // Go through every active raindrop and tick its timer down by one
        for(int i = 0; i < activeIndexes.size(); i++){
            timers.set(i, timers.get(i)-1);
            if(timers.get(i)<=0){
                timers.remove(i);
                activeIndexes.remove(i);
                dropColors.remove(i);
                i--;
            }
        }

        // If its not already at the max number of raindrops, add a few more
        if(activeIndexes.size() < maxVolume){
            for(int i = 0; i < speed; i++){
                activeIndexes.add((int)(Math.random()*bufferLength));
                timers.add((int)((Math.random()*50+1)/speed));
                dropColors.add(color==null?new int[] {(int)(Math.random()*256),(int)(Math.random()*256),(int)(Math.random()*256)}:color);
            }
        }

        for(int i = 0; i < activeIndexes.size(); i++){
            buffer.setRGB(activeIndexes.get(i), dropColors.get(i)[0], dropColors.get(i)[1], dropColors.get(i)[2]);
        }

        return buffer;
    }
}

