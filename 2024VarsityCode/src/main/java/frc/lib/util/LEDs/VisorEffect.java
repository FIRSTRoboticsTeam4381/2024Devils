package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

public class VisorEffect extends LightingEffect{
    private Color visorColor;
    private int visorLocation;
    private int visorSize;
    private int visorSpeed;

    private int movementModifier = -1;

    public VisorEffect(int startLED, int lastLED, Color visorColor, int visorSize, int speed, int startLocation){
        super(startLED, lastLED);
        this.visorColor = visorColor;
        this.visorSize = visorSize;
        this.visorSpeed = speed;
        visorLocation = startLocation;
    }
    public VisorEffect(int bufferLength, Color visorColor, int visorSize, int speed, int startLocation){
        this(0, bufferLength-1, visorColor, visorSize, speed, 0);
    }
    public VisorEffect(int startLED, int lastLED, Color visorColor, int visorSize, int speed){
        this(startLED, lastLED, visorColor, visorSize, speed, 0);
    }
    public VisorEffect(int bufferLength, Color visorColor, int visorSize, int speed){
        this(bufferLength, visorColor, visorSize, speed, 0);
    }

    @Override
    public Color[] updatePixels(){
        Color[] pixels = new Color[bufferLength];

        for(int i = 0; i < bufferLength; i++){
            if (i > visorLocation-visorSize && i < visorLocation+visorSize) pixels[i] = visorColor;
            else pixels[i] = new Color(0,0,0);
        }

        if(visorLocation <= 0 || visorLocation >= bufferLength-1){
            movementModifier *= -1;
        }
        visorLocation += movementModifier*visorSpeed;

        return pixels;
    }
}
