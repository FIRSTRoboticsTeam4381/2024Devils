package frc.lib.util.LEDs;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class RainEffect extends LightingEffect{
    private Color rainColor;
    private int rainSpeed;
    private int maxVolume;

    private ArrayList<Integer> activeIndexes;
    private ArrayList<Integer> timers;
    private ArrayList<Color> dropColors;

    public RainEffect(int startLED, int lastLED, int rainSpeed, int maxVolume){
        super(startLED, lastLED);
        this.rainSpeed = rainSpeed;
        this.maxVolume = maxVolume;

        activeIndexes = new ArrayList<Integer>();
        timers = new ArrayList<Integer>();
        dropColors = new ArrayList<Color>();
    }
    public RainEffect(int bufferLength, int rainSpeed, int maxVolume){
        this(0, bufferLength-1, rainSpeed, maxVolume);
    }
    public RainEffect(int startLED, int lastLED, Color rainColor, int rainSpeed, int maxVolume){
        this(startLED, lastLED, rainSpeed, maxVolume);
        this.rainColor = rainColor;
    }
    public RainEffect(int bufferLength, Color rainColor, int rainSpeed, int maxVolume){
        this(bufferLength, rainSpeed, maxVolume);
        this.rainColor = rainColor;
    }

    @Override
    public Color[] updatePixels(){
        Color[] pixels = new Color[bufferLength];

        for(int i = 0; i < bufferLength; i++){
            pixels[i] = new Color(0,0,0);
        }

        // Go through every active raindrop and tick its timer down by one
        for(int i = 0; i < activeIndexes.size(); i++){
            timers.set(i, timers.get(i)-1);
            if(timers.get(i)<=0){ // If a timer is at 0 ms, remove the raindrop
                timers.remove(i);
                activeIndexes.remove(i);
                dropColors.remove(i);
                i--;
            }
        }

        // If its not already at the max number of raindrops, add a few more
        if(activeIndexes.size() < maxVolume){
            for(int i = 0; i < rainSpeed; i++){
                activeIndexes.add((int)(Math.random()*bufferLength));
                timers.add((int)((Math.random()*50+1)/rainSpeed));
                dropColors.add(rainColor==null?new Color((int)(Math.random()*256),(int)(Math.random()*256),(int)(Math.random()*256)):rainColor);
            }
        }

        for(int i = 0; i < activeIndexes.size(); i++){
            pixels[activeIndexes.get(i)] = dropColors.get(i);
        }

        return pixels;
    }
}

