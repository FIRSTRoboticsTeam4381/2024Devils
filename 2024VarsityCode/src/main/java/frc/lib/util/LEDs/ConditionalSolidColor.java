package frc.lib.util.LEDs;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;

public class ConditionalSolidColor extends SolidColorEffect{
    private Supplier<Boolean> supplier;
    private Color falseColor;
    private Color trueColor;

    public ConditionalSolidColor(Supplier<Boolean> boolSupplier, Color trueColor, Color falseColor, int bufferLength){
        this(boolSupplier, trueColor, falseColor, 0, bufferLength-1);
    }
    public ConditionalSolidColor(Supplier<Boolean> boolSupplier, Color trueColor, Color falseColor, int startLED, int lastLED){
        super(startLED, lastLED, falseColor);
        supplier = boolSupplier;
        this.trueColor = trueColor;
        this.falseColor = falseColor;
    }

    @Override
    public Color[] updatePixels(){
        this.color = supplier.get() ? trueColor : falseColor;
        return super.updatePixels();
    }


}
