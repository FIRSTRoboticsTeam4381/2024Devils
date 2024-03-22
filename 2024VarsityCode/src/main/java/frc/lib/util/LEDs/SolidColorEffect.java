package frc.lib.util.LEDs;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;

public class SolidColorEffect extends LightingEffect{
    private Color color;
    private Supplier<Double> scaler;

    /**
     * 
     * @param location Location of the lighting effect within the LED zone. 0 is the first LED in the strip
     * @param length Length of the lighting effect
     * @param color The color to fill. Enter the color at max brightness, as the scaler will only reduce this color
     * @param brightnessScaler A double supplier that returns the percentage brightness to have this color display at
     * @param type Type, either cosmetic or status. Status effects are always shown on top
     */
    public SolidColorEffect(int location, int length, Color color, Supplier<Double> brightnessScaler, Type type){
        super(location, length, type);
        setColor(color);
        scaler = brightnessScaler;
    }

    /**
     * 
     * @param location Location of the lighting effect within the LED zone. 0 is the first LED in the strip
     * @param length Length of the lighting effect
     * @param color The color to fill
     * @param type Type, either cosmetic or status. Status effects are always shown on top
     */
    public SolidColorEffect(int location, int length, Color color, Type type){
        this(location, length, color, ()->{return 1.0;}, type);
    }

    public void setColor(Color c){
        this.color = c;
    }
    public Color getColor(){
        return color;
    }

    @Override
    public void update() {
        Color color = new Color(this.color.red*scaler.get(), this.color.green*scaler.get(), this.color.blue*scaler.get());


        /* If the recalculated color is the same as the last one, the scaler hasn't changed
         * so there's no need to adjust every pixel
         */
        if(!LEDZone.equal(color, pixels[0])){
            colorFill(color);
        }
    }
    
}
