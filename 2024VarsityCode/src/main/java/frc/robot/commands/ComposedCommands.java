package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class ComposedCommands {
    private Intake intake;
    private Index index;
    private Shooter shooter;
    private Pivot pivot;

    // TODO change pivot commands over to profiled motion once that's done

    public ComposedCommands(Intake intake, Index index, Shooter shooter, Pivot pivot){
        this.intake = intake;
        this.index = index;
        this.shooter = shooter;
        this.pivot = pivot;
    }


    /* INTAKE */

    /*
     * Should be able to run off of a toggleOnTrue because this will be cancelled by either other commands
     * or the toggle, which would stop the intake running as well as the index.
     */
    public Command groundIntake(){
        return new ParallelCommandGroup(
            pivot.goToIntake(),
            intake.run(), // Stops when cancelled
            index.indexUntilIn(false) // Stops when cancelled
        );
    }

    /*
     * Should also be able to run off of a toggleOnTrue for the same reasons, everything that
     * needs to stop should stop
     */
    public Command humanIntake(){
        return new ParallelCommandGroup(
            pivot.goToHuman(),
            index.indexUntilIn(true), // Stops when cancelled
            shooter.eject() // Stops when cancelled
        );
    }

    /* EJECT */

    /*
     * Should run off of holding a button, because cancelling on release should stop all three mechanisms
     */
    public Command ejectNote(){
        return new ParallelCommandGroup(
            shooter.eject(), // Stops when cancelled
            index.eject(), // Stops when cancelled
            intake.eject() // Stops when cancelled
        );
    }

    /* AMP MODE TOGGLE */

    public Command ampMode(){
        return new ParallelCommandGroup(
            pivot.goToTemporaryPosition(Pivot.AMP_POS), // Goes back to 0 when cancelled
            shooter.ampShoot() // Stops when cancelled
        );
    }

    /* FEED NOTE IF READY */

    /*
     * Should repeatedly check if the shooter is ready for the note while the button is held, and if it is it will
     * feed the note
     */
    public Command feedNote(){
        return new ConditionalCommand(index.run(), Commands.none(), shooter::readyForNote).repeatedly();
    }

    /* CANCEL ALL COMMANDS */
    public Command cancelAll(){
        return new ParallelCommandGroup(
            pivot.goToTransit(),
            shooter.instantStopAll(),
            index.instantStop(),
            intake.instantStop()
        );
    }
}