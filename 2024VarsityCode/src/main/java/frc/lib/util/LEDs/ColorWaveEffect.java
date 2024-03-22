package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;

public class ColorWaveEffect extends LightingEffect {
    private Color c1, c2;
    private double wavePeriod; // Length of the wave in pixels
    private double transitionPeriod; // Length of time for full transition between color in seconds/transition

    public ColorWaveEffect(int location, int length, Color c1, Color c2, double wavePeriod, double transitionPeriod, Type type){
        super(location, length, type);
        this.c1 = c1;
        this.c2 = c2;
        this.wavePeriod = wavePeriod;
        this.transitionPeriod = transitionPeriod;

        for(int i = 0; i < pixels.length; i++){
            pixels[i] = new Color(
                (int)(((c1.red-c2.red)/2.0)*Math.sin(2.0*Math.PI/wavePeriod * i) + ((c1.red+c2.red)/2.0)),
                (int)(((c1.green-c2.green)/2.0)*Math.sin(2.0*Math.PI/wavePeriod * i) + ((c1.green+c2.green)/2.0)),
                (int)(((c1.blue-c2.blue)/2.0)*Math.sin(2.0*Math.PI/wavePeriod * i) + ((c1.blue+c2.blue)/2.0))
            );
        }
    }
    public ColorWaveEffect(int location, int length, Type type) {
        super(location, length, type);
        //TODO Auto-generated constructor stub
    }

    @Override
    public void update() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }
    
}
