package frc.lib.util;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static NetworkTable limelightTable(){
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    public static boolean tagInView(){
        return limelightTable().getEntry("tv").getDouble(0) == 1 ? true : false;
    }

    public static double[] getPosition(){
        double[] pose = limelightTable().getEntry("botpose").getDoubleArray(new double[6]);
        double[] position = {pose[0],pose[1],pose[2]};
        return position;
    }
    public static double[] getBlueRelativePosition(){
        double[] pose = limelightTable().getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        double[] position = {pose[0],pose[1],pose[2]};
        return position;
    }
    public static double[] getRedRelativePosition(){
        double[] pose = limelightTable().getEntry("botpose_wpired").getDoubleArray(new double[6]);
        double[] position = {pose[0],pose[1],pose[2]};
        return position;
    }
}
