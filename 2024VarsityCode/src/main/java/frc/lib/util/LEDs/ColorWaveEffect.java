package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;

public class ColorWaveEffect extends LightingEffect {
    private Color c1, c2;
    private int wavePeriod, transPeriod;

    private Color[] wavePattern;
    private Color[] transPattern;

    int timer = 0;
    
    public ColorWaveEffect(int location, int length, Color c1, Color c2, int wavePeriod, int transitionPeriod, Type type){
        super(location, length, type);
        this.c1=c1;
        this.c2=c2;
        this.wavePeriod=wavePeriod;
        this.transPeriod=transitionPeriod/4;

        wavePattern = new Color[wavePeriod];
        transPattern = new Color[transPeriod];

        double[] waveSlopes = new double[] {(c2.red-c1.red)/(wavePeriod/2.0),(c2.green-c1.green)/(wavePeriod/2.0),(c2.blue-c1.blue)/(wavePeriod/2.0)};
        double[] transSlopes = new double[] {(c2.red-c1.red)/(transPeriod/2.0),(c2.green-c1.green)/(transPeriod/2.0),(c2.blue-c1.blue)/(transPeriod/2.0)};

        /* Wave Pattern */
        for(int i = 0; i < wavePeriod/2; i++){
            wavePattern[i] = new Color(waveSlopes[0]*i+c1.red,waveSlopes[1]*i+c1.green,waveSlopes[2]*i+c1.blue);
        }
        for(int i = wavePeriod-1; i>= wavePeriod/2; i--){
            wavePattern[i] = wavePattern[wavePeriod-1-i];
        }
        for(int i = 0; i < wavePeriod; i++){
            if(wavePattern[i]==null)wavePattern[i]=new Color((wavePattern[i-1].red+wavePattern[i+1].red)/2.0,(wavePattern[i-1].green+wavePattern[i+1].green)/2.0,(wavePattern[i-1].blue+wavePattern[i+1].blue)/2.0);
        }
        /* At this point, wavePattern[] contains the pattern of colors for a smooth transition
        * back and forth in the number of steps defined by wavePeriod,
        * so at this point this array just needs to be printed on loop
         */

        /* Transition Pattern */
        for(int i = 0; i < transPeriod/2; i++){
            transPattern[i] = new Color(transSlopes[0]*i+c1.red,transSlopes[1]*i+c1.green,transSlopes[2]*i+c1.blue);
        }
        for(int i = transPeriod-1; i>= transPeriod/2; i--){
            transPattern[i] = transPattern[transPeriod-1-i];
        }
        for(int i = 0; i < transPeriod; i++){
            if(transPattern[i]==null)transPattern[i]=new Color((transPattern[i-1].red+transPattern[i+1].red)/2.0,(transPattern[i-1].green+transPattern[i+1].green)/2.0,(transPattern[i-1].blue+transPattern[i+1].blue)/2.0);
        }
        /* At this point, transPattern[] contains the pattern of colors for a smooth transition
         * back and forth in the number of steps defined by transPeriod,
         * so at this point this array just needs to be printed on loop
        */
        System.out.println("Test");
    }

    @Override
    public void update() {
        for(int i = 0; i < length; i++){
            int relativePosition = (int)(((i%wavePeriod))/(double)wavePeriod*transPeriod);
            int j = (relativePosition+timer)%transPeriod;
            pixels[i] = transPattern[j];
        }
        timer+=20;
    }

    
}
