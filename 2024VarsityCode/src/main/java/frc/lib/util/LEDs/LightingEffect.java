package frc.lib.util.LEDs;

import edu.wpi.first.wpilibj.util.Color;

public abstract class LightingEffect {
    protected int location, length;
    protected Type type;
    protected Color[] pixels;

    /**
     * 
     * @param location Location of the lighting effect within the LED zone. 0 is the first LED in the strip
     * @param length Length of the lighting effect
     * @param type Type, either cosmetic or status. Status effects are always shown on top
     */
    protected LightingEffect(int location, int length, Type type){
        this.type = type;
        setLocation(location);
        setLength(length);
        pixels = new Color[length];
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

    protected void colorFill(Color c){
        for(int i = 0; i < pixels.length; i++){
            pixels[i] = c;
        }
    }

    public abstract void update();

    public enum Type{
        cosmetic,
        status
    }
}
