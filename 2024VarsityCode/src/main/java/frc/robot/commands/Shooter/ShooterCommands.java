package frc.robot.commands.Shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    private Shooter s_Shooter;
    public ShooterCommands(Shooter shooter){
        s_Shooter = shooter;
    }

    public ConditionalCommand toggleShooter(double speed, Shooter.Mode mode){
        return new ConditionalCommand(
            new ShooterRampUp(s_Shooter, speed, 0.1, mode), 
            new InstantCommand(() -> s_Shooter.setAll(0, mode)), 
            this::getStopped
        );
    }

    private boolean getStopped(){
        return s_Shooter.getSetSpeed()==0;
    }
}
