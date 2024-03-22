package frc.lib.util.LEDs;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;

public class VisorEffect extends LightingEffect{
    private double size, speed;
    private double location;
    private int directionModifier = 1;
    private Supplier<Double> spdSclr, szeSclr;
    private Color color;

    /**
     * 
     * @param location Location of the lighting effect within the LED zone. 0 is the first LED in the strip
     * @param length Length of the lighting effect
     * @param color The color of the visor
     * @param maxSize The MAXIMUM size of the visor
     * @param startLocation The pixel for the center of the visor to start at the beginning of the effect
     * @param maxSpeed The MAXIMUM speed of the visor in pixels/second
     * @param speedScaler A double supplier that returns the percentage of the maxSpeed the visor should move at during a given frame
     * @param sizeScaler A double supplier that returns the percentage of the maxSize the visor should have during a given frame
     * @param type Type, either cosmetic or status. Status effects are always shown on top
     */
    public VisorEffect(int location, int length, Color color, int maxSize, int startLocation, double maxSpeed, Supplier<Double> speedScaler, Supplier<Double> sizeScaler, Type type){
        super(location, length, type);
        this.size = maxSize;
        this.speed = maxSpeed/1000.0*20.0;
        spdSclr = speedScaler;
        szeSclr = sizeScaler;
        location = startLocation;
        this.color = color;
    }
    /**
     * 
     * @param location Location of the lighting effect within the LED zone. 0 is the first LED in the strip
     * @param length Length of the lighting effect
     * @param color The color of the visor
     * @param size The size of the visor
     * @param startLocation The pixel for the center of the visor to start at the beginning of the effect
     * @param speed The speed of the visor in pixels/second
     * @param type Type, either cosmetic or status. Status effects are always shown on top
     */
    public VisorEffect(int location, int length, Color color, int size, int startLocation, double speed, Type type){
        this(location, length, color, size, startLocation, speed, ()->{return 1.0;}, ()->{return 1.0;}, type);
    }

    @Override
    public void update() {
        if(location<0||location>=length){
            directionModifier*=-1;
        }

        double speed = this.speed*spdSclr.get();
        double size = this.size*szeSclr.get();

        location+=(speed*directionModifier);

        colorFill(new Color(0,0,0));

        for(int i = (int)(location-(size/2.0)); i < (int)(location+(size/2.0)); i++){
            if(i>=0&&i<length){
                pixels[i] = color;
            }
        }
    }
    
}
