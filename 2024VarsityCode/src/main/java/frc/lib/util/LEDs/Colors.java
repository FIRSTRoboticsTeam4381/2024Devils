package frc.lib.util.LEDs;

public class Colors {
    public static int[] RED = {255, 0, 0};
    public static int[] GREEN = {0, 255, 0};
    public static int[] BLUE = {0, 0, 255};
    public static int[] YELLOW = {255, 255, 0};
    public static int[] PURPLE = {255, 0, 255};
    public static int[] TEAL = {0, 255, 255};
    public static int[] OFF = {0, 0, 0};
    public static int[] WHITE = {255, 255, 255};

    public static int[][] stepMorph(int[] current, int[] target, int steps){
        int rStep = Math.abs(current[0] - target[0])/steps;
        int gStep = Math.abs(current[1] - target[1])/steps;
        int bStep = Math.abs(current[2] - target[2])/steps;

        int[][] stepArray = new int[steps][3];
        for(int i = 0; i < steps; i++){
            stepArray[i][0] = target[0]>current[0]?current[0]+rStep*(i+1):current[0]-rStep*(i+1);
            stepArray[i][1] = target[1]>current[1]?current[1]+rStep*(i+1):current[1]-rStep*(i+1);
            stepArray[i][2] = target[2]>current[2]?current[2]+rStep*(i+1):current[2]-rStep*(i+1);
        }
        return stepArray;
    }
}
