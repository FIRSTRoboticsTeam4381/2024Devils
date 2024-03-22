package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;

public abstract class LightingEffect {
    protected int location, length;
    protected Type type;
    protected Color[] pixels;

    public LightingEffect(int location, int length, Type type){
        this.type = type;
        setLocation(location);
        setLength(length);
    }

    public void setLocation(int l){
        location = l;
    }
    public void setLength(int l){
        length = l;
    }

    public int getLocation(){
        return location;
    }
    public int getLength(){
        return length;
    }
    public Type getType(){
        return type;
    }

    public Color[] getPixels(){
        return pixels;
    }

    public abstract void update();

    public enum Type{
        cosmetic,
        status
    }
}
