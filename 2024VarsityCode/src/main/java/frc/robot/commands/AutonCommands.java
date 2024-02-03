package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonCommands {
    public static SequentialCommandGroup aimAndFire(){
        return new SequentialCommandGroup(
            Commands.none(), // TODO start shooter
            Commands.none(), // TODO aim shooter
            Commands.none()  // TODO index until shot
        );
    }
    public static SequentialCommandGroup startIntaking(){
        return new SequentialCommandGroup();
    }
    public static SequentialCommandGroup stopIntaking(){
        return new SequentialCommandGroup();
    }
}
