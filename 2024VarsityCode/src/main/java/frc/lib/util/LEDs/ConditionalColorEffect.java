package frc.lib.util.LEDs;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;

public class ConditionalColorEffect extends LightingEffect{
    private Supplier<Boolean> boolSupplier;
    private Color trueColor, falseColor;

    public ConditionalColorEffect(int location, int length, Supplier<Boolean> boolSupplier, Color trueColor, Color falseColor, Type type){
        super(location, length, type);
        this.boolSupplier=boolSupplier;
        this.trueColor=trueColor;
        this.falseColor=falseColor;
    }

    @Override
    public void update() {
        if(boolSupplier.get()){
            colorFill(trueColor);
        }else{
            colorFill(falseColor);
        }
    }
    
}
