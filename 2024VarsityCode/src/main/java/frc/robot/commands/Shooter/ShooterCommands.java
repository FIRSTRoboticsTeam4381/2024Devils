package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Index.IndexUntilShot;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
    private Shooter s_Shooter;
    private Indexer s_Index;

    private static final int shooterError = 200;

    private boolean shooterRunning = false;
    private boolean autoAiming = false;

    public ShooterCommands(Shooter shooter, Indexer index){
        this.s_Shooter = shooter;
        this.s_Index = index;
    }

    public SequentialCommandGroup startShooter(double percSpeed){
        shooterRunning = true;
        return new SequentialCommandGroup(
            new ShooterRamp(s_Shooter, percSpeed*Shooter.maxRPM, shooterError),
            new InstantCommand(() -> s_Shooter.setSpeed(percSpeed), s_Shooter)
        );
    }
    public InstantCommand stopShooter(){
        shooterRunning = false;
        return new InstantCommand(() -> s_Shooter.setSpeed(0), s_Shooter);
    }

    public ConditionalCommand toggleShooter(){
        return new ConditionalCommand(
            stopShooter(), 
            startShooter(0.75), 
            () -> getShooterRunning());
    }

    public ConditionalCommand feedNote(){
        return new ConditionalCommand(
            new IndexUntilShot(s_Index),                 // Command when true
            Commands.print("Not Ready"),         // Command when false     
            () -> s_Shooter.isWithinError(shooterError)  // BoolSupplier run condition
        );
    }

    public boolean getShooterRunning(){
        return shooterRunning;
    }
    public boolean getAutoAiming(){
        return autoAiming;
    }
}
