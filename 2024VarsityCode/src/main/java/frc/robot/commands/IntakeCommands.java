package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Index.IndexUntilIn;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;

public class IntakeCommands {
    private Intake intake;
    private Indexer indexer;
    private ShooterPivot shooterPivot;

    private double defaultIntakeSpeed = 0.4;

    private boolean intaking = false;

    public IntakeCommands(Intake intake, Indexer indexer, ShooterPivot shooterPivot){
        this.intake = intake;
        this.indexer = indexer;
        this.shooterPivot = shooterPivot;
    }

    /**
     * Command to move the pivot up, start the intake motors, run intake until note is sensed
     * by the break beam (ending the indexer command), and stop the intake
     * @return
     */
    public SequentialCommandGroup intakeUntilIn(){
        intaking = true;
        return new SequentialCommandGroup(
            // Move shooterpivot to intake position,
            new InstantCommand(() -> intake.setIntakeSpeed(defaultIntakeSpeed), intake),
            new IndexUntilIn(indexer),
            stopAll()
        );
    }

    public ParallelCommandGroup stopAll(){
        return new ParallelCommandGroup(
            // Move shooterpivot back to home
            new InstantCommand(() -> intake.setIntakeSpeed(0), intake),
            new InstantCommand(() -> indexer.setSpeed(0), indexer)
        );
    }

    public ConditionalCommand toggleIntake(){
        return new ConditionalCommand(stopAll(), intakeUntilIn(), () -> getIntaking());
    }

    private boolean getIntaking(){
        return intaking;
    }
}
