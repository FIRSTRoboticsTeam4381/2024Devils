package frc.lib.util.LEDs;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.util.Color;

public class RainEffect extends LightingEffect{
    private double maxTime; // Max time in milliseconds
    private int maxVolume;
    private Color color;

    private ArrayList<RainDrop> rain;

    /**
     * Creates a new rain effect with all raindrops sharing the same color
     * @param location Location of the lighting effect within the LED zone. 0 is the first LED in the strip
     * @param length Length of the lighting effect
     * @param maxTime The maximum time in milliseconds for raindrops to remain.
     * @param maxVolume The maximum number of raindrops that can exist at once.
     * @param color The color for all raindrops to have
     * @param type Type, either cosmetic or status. Status effects are always shown on top
     */
    public RainEffect(int location, int length, double maxTime, int maxVolume, Color color, Type type){
        this(location, length, maxTime, maxVolume, type);
        this.color = color;
    }
    /**
     * Creates a new rain effect with all raindrops having randomized colors
     * @param location Location of the lighting effect within the LED zone. 0 is the first LED in the strip
     * @param length Length of the lighting effect
     * @param maxTime The maximum time in milliseconds for raindrops to remain.
     * @param maxVolume The maximum number of raindrops that can exist at once.
     * @param type Type, either cosmetic or status. Status effects are always shown on top
     */
    public RainEffect(int location, int length, double maxTime, int maxVolume, Type type){
        super(location, length, type);
        this.maxTime = maxTime;
        this.maxVolume = maxVolume;
        color = null;
        rain = new ArrayList<RainDrop>();
    }


    @Override
    public void update() {
        // Set all pixels to off
        colorFill(new Color(0,0,0));

        // Go through every pixel and tick down the timer & remove any pixels that have run out
        for(int i = 0; i < rain.size(); i++){
            rain.get(i).tick();
            if(rain.get(i).getTimer()<0){
                rain.remove(i);
                i--;
            }
        }

        // If not at max volume, add more
        if(rain.size()<maxVolume){
            rain.add(new RainDrop(
                (int)(Math.random()*length), 
                (int)(Math.random()*maxTime), 
                color==null ?
                    new Color((int)(Math.random()*256), (int)(Math.random()*256), (int)(Math.random()*256)) :
                    color));
        }

        // Add raindrops to the pixel array
        for(RainDrop r : rain){
            pixels[r.getIndex()] = r.getColor();
        }
    }
    

    private class RainDrop{
        private int index, timer;
        private Color color;

        public RainDrop(int index, int timer, Color color){
            this.index = index;
            this.timer = timer;
            this.color = color;
        }
        public void tick(){
            timer--;
        }
        public int getTimer(){
            return timer;
        }
        public int getIndex(){
            return index;
        }
        public Color getColor(){
            return color;
        }
    }
}
