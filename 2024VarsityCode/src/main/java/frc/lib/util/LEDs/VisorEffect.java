package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class VisorEffect extends LightingEffect{
    private int location;
    private int[][] pixels;

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

        pixels = new int[bufferLength][3];
    }
    public VisorEffect(int bufferLength, int[] visorColor, int visorSize, int speed){
        this(bufferLength, visorColor, visorSize, speed, 0);
    }

    public AddressableLEDBuffer updateBuffer(){
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(bufferLength);

        for(int i = location; i < location+visorSize; i++){
            if(!(i > pixels.length-1) && !(i < 0)){
                pixels[i] = visorColor;
            }
        }
        for(int i = location; i > location-visorSize; i--){
            if(!(i < 0) && !(i > pixels.length-1)){
                pixels[i] = visorColor;
            }
        }
        if(location <= 0 || location >= pixels.length-1){
            modifier *= -1;
        }
        location += modifier*speed;

        for(int i = 0; i < bufferLength; i++){
            buffer.setRGB(i, pixels[i][0], pixels[i][1], pixels[i][2]);
        }

        return buffer;
    }
}
