package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    private Shooter shooter;
    public ShooterCommands(Shooter shooter){
        this.shooter = shooter;
    }

    private InstantCommand runShooters(double speed){
        return new InstantCommand(() -> shooter.setShooterSpeeds(speed));
    }
    private InstantCommand runTopDirector(double speed){
        return new InstantCommand(() -> shooter.setTopDirectionalSpeed(speed));
    }
    private InstantCommand runBottomDirector(double speed){
        return new InstantCommand(() -> shooter.setBottomDirectionalSpeed(speed));
    }

    public ParallelCommandGroup stopShooter(){
        return new ParallelCommandGroup(
            runShooters(0),
            runTopDirector(0),
            runBottomDirector(0)
        );
    }
    public ParallelCommandGroup runShooter(){
        return new ParallelCommandGroup(
            runShooters(1),
            runTopDirector(1),
            runBottomDirector(1)
        );
    }
    public ParallelCommandGroup runTopOutput(){
        return new ParallelCommandGroup(
            runShooters(0.25),
            runTopDirector(-0.25),
            runBottomDirector(0.25)
        );
    }
    public ParallelCommandGroup runBottomOutput(){
        return new ParallelCommandGroup(
            runShooters(0.25),
            runTopDirector(0.25),
            runBottomDirector(-0.25)
        );
    }
}
