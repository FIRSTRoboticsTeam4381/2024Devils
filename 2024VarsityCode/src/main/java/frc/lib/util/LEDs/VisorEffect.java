package frc.lib.util.LEDs;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;

public class VisorEffect extends LightingEffect{
    private Color color;
    private double location;
    private Supplier<Double> spdSclr;
    private Supplier<Double> szSclr;
    private double maxSpeed; // Frequency in pixels/second
    private double periodicFreq; // Frequency in pixels/20ms
    private double maxSize; // Size in pixels
    private int directionModifier = 1;


    public VisorEffect(int firstLED, int lastLED, Color color, double startLocation, double maxSpeed, double maxSize, Supplier<Double> speedScaler, Supplier<Double> sizeScaler, Type type){
        super(firstLED, lastLED, type);
        this.color = color;
        location = startLocation;
        this.spdSclr = speedScaler;
        this.szSclr = sizeScaler;
        this.maxSize = maxSize;
        this.maxSpeed = maxSpeed;
        periodicFreq = maxSpeed/1000.0*20.0;
    }
    public VisorEffect(int firstLED, int lastLED, Color color, double startLocation, double speed, double size, Type type){
        this(firstLED, lastLED, color, startLocation, speed, size, ()->{return 1.0;}, ()->{return 1.0;}, type);
    }
    public VisorEffect(int bufferLength, Color color, double startLocation, double maxSpeed, double maxSize, Supplier<Double> speedScaler, Supplier<Double> sizeScaler, Type type){
        this(0, bufferLength-1, color, startLocation, maxSpeed, maxSize, speedScaler, sizeScaler, type);
    }
    public VisorEffect(int bufferLength, Color color, double startLocation, double speed, double size, Type type){
        this(bufferLength, color, startLocation, speed, size, ()->{return 1.0;}, ()->{return 1.0;}, type);
    }

    @Override
    public Color[] updatePixels() {
        double speed = maxSpeed * spdSclr.get();
        double size = maxSize * szSclr.get();

        if(location<0 || location>bufferLength) {directionModifier*=-1;}

        location += speed*directionModifier;

        double[] visorRange = new double[] {location-(size/2.0), location+(size/2.0)};

        Color[] pixels = new Color[bufferLength];
        for(int i = 0; i < pixels.length; i++){
            if(i>=visorRange[0] && i<=visorRange[1]){
                pixels[i]=color;
            }else{
                pixels[i]=new Color(0,0,0);
            }
        }
        return pixels;
    }
    
}
