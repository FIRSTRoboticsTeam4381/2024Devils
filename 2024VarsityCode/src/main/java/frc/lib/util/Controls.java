package frc.lib.util;

import java.util.function.Supplier;

public class Controls {
    public static class JoystickLimiter{
        private Supplier<Double> joystickInput;
        private double acceleration;
        private double output;
        public JoystickLimiter(Supplier<Double> joystickInput, double maxAcceleration){
            output=0.0;
            this.joystickInput=joystickInput;
            this.acceleration=Math.abs(maxAcceleration);
        }
        public double get(){
            double input = -joystickInput.get();
            if(Math.abs(input-output)<acceleration){
                output=input;
            }
            else{
                output += (input-output>0?acceleration:-acceleration);
            }
            return output;
        }

    }
}
