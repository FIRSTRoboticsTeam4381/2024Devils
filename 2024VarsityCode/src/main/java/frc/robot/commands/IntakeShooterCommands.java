package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Index.IndexUntilIn;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class IntakeShooterCommands {
    private Intake intake;
    private Indexer indexer;
    private Shooter shooter;
    private ShooterPivot shooterPivot;

    private double defaultIntakeSpeed = 0.4;

    public IntakeShooterCommands(Intake intake, Indexer indexer, Shooter shooter, ShooterPivot shooterPivot){
        this.intake = intake;
        this.indexer = indexer;
        this.shooter = shooter;
        this.shooterPivot = shooterPivot;
    }

    /**
     * Command to move the pivot up, start the intake motors, run intake until note is sensed
     * by the break beam (ending the indexer command), and stop the intake
     * @return
     */
    public SequentialCommandGroup intakeUntilIn(){
        return new SequentialCommandGroup(
            // Move shooterpivot to intake position,
            new InstantCommand(() -> intake.setIntakeSpeed(defaultIntakeSpeed), intake),
            new IndexUntilIn(indexer),
            new InstantCommand(() -> intake.setIntakeSpeed(0), intake)
            // Move shooterpivot back to home
        );
    }
}
