package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisorEffect extends LightingEffect{
    private int location;

    private int[] visorColor;

    private int visorSize;
    private int speed;

    private int modifier = -1;

    public VisorEffect(int bufferLength, int[] visorColor, int visorSize, int speed, int startLocation){
        super(bufferLength);
        this.visorColor = visorColor;
        this.visorSize = visorSize;
        this.speed = speed;
        location = startLocation;
    }
    public VisorEffect(int bufferLength, int[] visorColor, int visorSize, int speed){
        this(bufferLength, visorColor, visorSize, speed, 0);
    }

    public AddressableLEDBuffer updateBuffer(){
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(bufferLength);

        for(int i = 0; i < bufferLength; i++){
            if (i > location-visorSize && i < location+visorSize) buffer.setRGB(i, visorColor[0], visorColor[1], visorColor[2]);
            else buffer.setRGB(i, 0, 0, 0);
        }

        if(location <= 0 || location >= bufferLength-1){
            modifier *= -1;
        }
        location += modifier*speed;

        return buffer;
    }
}
