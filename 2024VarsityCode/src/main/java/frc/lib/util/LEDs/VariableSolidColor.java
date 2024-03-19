package frc.lib.util.LEDs;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;

public class VariableSolidColor extends SolidColorEffect{
    private Supplier<Double> supplier;

    public VariableSolidColor(Supplier<Double> variableSupplier, int bufferLength, Color color){
        this(variableSupplier, 0, bufferLength-1, color);
    }
    public VariableSolidColor(Supplier<Double> variableSupplier, int startLED, int lastLED, Color color){
        super(startLED, lastLED, color);
        supplier = variableSupplier;
    }

    public Color[] updateAtReference(double reference){
        return updatePixelsAtPercent(Math.min(reference, supplier.get())/Math.max(reference, supplier.get()));
    }
}
