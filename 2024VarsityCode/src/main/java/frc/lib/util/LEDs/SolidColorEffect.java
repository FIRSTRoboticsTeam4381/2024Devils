package frc.lib.util.LEDs;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;

public class SolidColorEffect extends LightingEffect{
    private Color color;
    private Supplier<Double> scaler;

    public SolidColorEffect(int location, int length, Color color, Supplier<Double> brightnessScaler, Type type){
        super(location, length, type);
        setColor(color);
        scaler = brightnessScaler;
    }

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
        color = new Color(color.red*scaler.get(), color.green*scaler.get(), color.blue*scaler.get());
        if(!LEDZone.equal(color, pixels[0])){
            for(int i = 0; i < length; i++){
                pixels[i] = color;
            }
        }
    }
    
}
