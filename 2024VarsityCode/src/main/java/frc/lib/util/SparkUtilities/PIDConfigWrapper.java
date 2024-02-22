package frc.lib.util.SparkUtilities;

import com.revrobotics.CANSparkBase;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;

public class PIDConfigWrapper {
    private SparkPIDController controller;
    
    public PIDConfigWrapper(CANSparkBase b){
        controller = b.getPIDController();
    }

    public PIDConfigWrapper setFeedbackDevice(MotorFeedbackSensor sensor){
        controller.setFeedbackDevice(sensor);
        return this;
    }
    public PIDConfigWrapper setOutputRange(double min, double max){
        controller.setOutputRange(min, max);
        return this;
    }

    public PIDConfigWrapper setPositionWrapping(boolean enabled, double min, double max){
        setPositionWrappingEnabled(enabled);
        setPositionWrappingMin(min);
        setPositionWrappingMax(max);
        return this;
    }
    public PIDConfigWrapper setPositionWrappingEnabled(boolean enabled){
        controller.setPositionPIDWrappingEnabled(enabled);
        return this;
    }
    public PIDConfigWrapper setPositionWrappingMin(double min){
        controller.setPositionPIDWrappingMinInput(min);
        return this;
    }
    public PIDConfigWrapper setPositionWrappingMax(double max){
        controller.setPositionPIDWrappingMaxInput(max);
        return this;
    }

    public PIDConfigWrapper setPIDF(double kp, double ki, double kd, double kff){
        setP(kp);
        setI(ki);
        setD(kd);
        setFF(kff);
        return this;
    }
    public PIDConfigWrapper setP(double kp){
        controller.setP(kp);
        return this;
    }
    public PIDConfigWrapper setI(double ki){
        controller.setI(ki);
        return this;
    }
    public PIDConfigWrapper setD(double kd){
        controller.setD(kd);
        return this;
    }
    public PIDConfigWrapper setFF(double kf){
        controller.setFF(kf);
        return this;
    }

    public SparkPIDController get(){
        return controller;
    }
}
