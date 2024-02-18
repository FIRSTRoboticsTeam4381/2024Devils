package frc.lib.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class SparkOptimizer {
    public static void optimizeFrames(CANSparkBase s, boolean hasFollowers, boolean needVelocity, boolean needPosition, boolean usingAnalog, boolean usingAlternate, boolean usingAbsolute){
        s.setPeriodicFramePeriod(PeriodicFrame.kStatus0, hasFollowers?10:50);
        s.setPeriodicFramePeriod(PeriodicFrame.kStatus1, needVelocity?20:50);
        s.setPeriodicFramePeriod(PeriodicFrame.kStatus2, needPosition?10:200);
        s.setPeriodicFramePeriod(PeriodicFrame.kStatus3, usingAnalog?10:2000);
        s.setPeriodicFramePeriod(PeriodicFrame.kStatus4, usingAlternate?10:2000);
        s.setPeriodicFramePeriod(PeriodicFrame.kStatus5, usingAbsolute?10:2000);
        s.setPeriodicFramePeriod(PeriodicFrame.kStatus6, usingAbsolute?10:2000);
    }
}
