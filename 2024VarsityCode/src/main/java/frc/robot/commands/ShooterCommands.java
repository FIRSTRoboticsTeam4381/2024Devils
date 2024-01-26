package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    private Shooter shooter;
    private CommandPS4Controller controller;

    public ShooterCommands(Shooter shooter, CommandPS4Controller controller){
        this.shooter = shooter;
        this.controller = controller;
    }
}
