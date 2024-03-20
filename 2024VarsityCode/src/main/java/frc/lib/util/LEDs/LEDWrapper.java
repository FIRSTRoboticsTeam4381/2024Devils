package frc.lib.util.LEDs;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.LEDs.LegacyEffects.SolidColorEffect;
import frc.lib.util.LEDs.LightingEffect.Type;

public class LEDWrapper {
    private AddressableLED ledStrip;
    private int length;
    private ArrayList<LightingEffect> activeEffects;
    private AddressableLEDBuffer buffer;

    
    public LEDWrapper(int port, int length){
        ledStrip = new AddressableLED(port);
        setLength(length);

        activeEffects = new ArrayList<LightingEffect>();
        clearEffects();
        ledStrip.start();
    }

    public void periodic(){
        buffer = toLEDBuffer(updatePixels());
        ledStrip.setData(buffer);
    }

    public Color[] updatePixels(){
        Color[] pixels = activeEffects.get(0).updatePixels();

        for(int i = 1; i < activeEffects.size(); i++){
            Color[] effectPixels = activeEffects.get(i).updatePixels();

            for(int j = activeEffects.get(i).getFirstLED(); j<=activeEffects.get(i).getLastLED();j++){
                if(!sameColor(effectPixels[j], new Color(0,0,0))){
                    pixels[j]=effectPixels[j-activeEffects.get(i).getFirstLED()];
                }
            }
        }
        return pixels;
    }

    public boolean sameColor(Color c1, Color c2){
        return c1.red==c2.red && c1.green==c2.green && c1.blue==c2.blue;
    }

    public AddressableLEDBuffer toLEDBuffer(Color[] array){
        AddressableLEDBuffer buffer = new AddressableLEDBuffer(array.length);
        for(int i = 0; i < array.length; i++){
            buffer.setRGB(i, (int) array[i].red, (int) array[i].green, (int) array[i].blue);
        }
        return buffer;
    }

    public void addEffect(LightingEffect e){
        if(e.getType()==Type.cosmetic){
            if(activeEffects.get(activeEffects.size()-1).getType()==Type.cosmetic){
                activeEffects.add(e);
            }else{
                for(int i = activeEffects.size()-2; i>=0; i++){
                    if(activeEffects.get(i).getType()==Type.cosmetic){
                        activeEffects.add(i+1, e);
                    }
                }
            }
        }
        activeEffects.add(e);
    }
    public void clearEffects(){
        activeEffects.clear();
        addEffect(new SolidColorEffect(length, new Color(0,0,0), Type.cosmetic));
    }
    public void setEffects(LightingEffect... es){
        clearEffects();
        for(LightingEffect e : es){
            addEffect(e);
        }
    }

    public int getLength(){
        return length;
    }
    public void setLength(int l){
        if(l<0) l=0;
        else if(l>500) l=500;
        length=l;
        ledStrip.setLength(l);
    }
}
