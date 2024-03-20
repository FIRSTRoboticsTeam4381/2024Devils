package frc.lib.util.LEDs;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;

public class LEDWrapper {
    private AddressableLED ledStrip;
    private int length;
    private ArrayList<LightingEffect> activeEffects;
    
    public LEDWrapper(int port, int length){
        ledStrip = new AddressableLED(port);
        setLength(length);

        activeEffects = new ArrayList<LightingEffect>();
    }

    public void clearEffects(){
        activeEffects.clear();
        activeEffects.add(new SolidColorEffect(length, new Color(0,0,0)));
    }
    public void addEffect(LightingEffect e){
        if(e.getType()==LightingEffect.Type.cosmetic){
            if(activeEffects.get(activeEffects.size()-1).getType()==LightingEffect.Type.cosmetic){
                activeEffects.add(e);
            }else{
                for(int i = activeEffects)
            }
        }
        activeEffects.add(e);
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
