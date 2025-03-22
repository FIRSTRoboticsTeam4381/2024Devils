package frc.robot.commands;

import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class ComposedCommands {
    private Intake intake;
    private Index index;
    private Shooter shooter;
    private Pivot pivot;
    private Climb climb;
    private Limelight ll;
    private Swerve swerve;
    private CommandPS4Controller controller;
    private State state = State.None;

    // TODO change pivot commands over to profiled motion once that's done

    public ComposedCommands(CommandPS4Controller controller, Intake intake, Index index, Shooter shooter, Pivot pivot, Climb climb, Limelight ll, Swerve swerve){
        this.intake = intake;
        this.index = index;
        this.shooter = shooter;
        this.pivot = pivot;
        this.ll = ll;
        this.swerve = swerve;
        this.climb=climb;
        this.controller = controller;
    }

    public Command setRobotState(State state){
        return new InstantCommand(()->this.state=state);
    }


    /* INTAKE */
    public Command groundIntake(Command handoff){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                index.indexUntilIn(false),
                new ParallelCommandGroup( // Just get the note into the index
                    pivot.goToAngle(Pivot.Positions.INTAKE, ClosedLoopSlot.kSlot1),
                    intake.run()
                )
            ),
            
            new ParallelRaceGroup( // Get note up to the shooter wheels
                handoff,
                index.indexUntilReady(false)
            )
        ).withName("Ground Intake");
    }

    public Command humanIntake(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                pivot.goToAngle(Pivot.Positions.HUMAN, ClosedLoopSlot.kSlot0),
                index.indexUntilIn(true), // Stops when cancelled
                shooter.eject() // Stops when cancelled
            ),
            new ParallelCommandGroup(
                pivot.goToAngle(Pivot.Positions.TRANSIT, ClosedLoopSlot.kSlot0),
                index.indexUntilReady(false)
            )
        ).withName("Human Intake");
    }

    /* EJECT */

    public Command ejectNote(){
        return new ParallelCommandGroup(
            shooter.eject(), // Stops when cancelled
            index.eject(), // Stops when cancelled
            intake.eject() // Stops when cancelled
        ).withName("Eject Note");
    }

    public Command reverseAmp(){
        return new ParallelCommandGroup(
            shooter.ejectFromAmp(),
            index.eject()
        ).withName("Reverse Amp");
    }

    /* AMP MODE TOGGLE */
    public Command ampMode(){
            return new ParallelCommandGroup(
                pivot.goToAngle(Pivot.Positions.AMP, ClosedLoopSlot.kSlot0),
                shooter.ampShoot(),
                setRobotState(State.Amp)
            ).withName("Amp Mode");
    }

    /* PODIUM MODE */
    public Command podiumMode(){
            return new ParallelCommandGroup(
                pivot.goToAngle(32, ClosedLoopSlot.kSlot1),
                shooter.shoot(4670),
                setRobotState(State.Podium)
            ).withName("Podium Mode");
    }

    /* SUBWOOFER MODE */
    public Command subwooferMode(){
            return new ParallelCommandGroup(
                pivot.goToAngle(49.5, ClosedLoopSlot.kSlot1),
                shooter.shoot(3625),
                setRobotState(State.Subwoofer)
            ).withName("Subwoofer Mode");
    }

    /* ALLIANCE LINE MODE */
    public Command allianceLineMode(){
            return new ParallelCommandGroup(
                pivot.goToAngle(35, ClosedLoopSlot.kSlot1),
                shooter.shoot(4475),
                setRobotState(State.Alliance)
            ).withName("Alliance Line Mode");
    }
    

    /* START SHOOTER */
    public Command startShooter(){
        return new ParallelCommandGroup(
            shooter.shoot(5000)
        ).withName("Shooter Running");
    }

    /* TODO check positions and sequence */
    public Command climb(){
        return new SequentialCommandGroup(
            climb.goToPosition(0.674, ClosedLoopSlot.kSlot0),
            pivot.goToAngle(18.5, ClosedLoopSlot.kSlot1),
            climb.goToPosition(0.370, ClosedLoopSlot.kSlot0),
            new ParallelCommandGroup(
                pivot.goToAngle(90.0, ClosedLoopSlot.kSlot1),
                climb.goToPosition(0.02, ClosedLoopSlot.kSlot0)
            )
        ).withName("Climbing");
    }

    /* FEED NOTE IF READY */

    /*
     * Should repeatedly check if the shooter is ready for the note while the button is held, and if it is it will
     * feed the note
     */
    public Command feedNote(){
        return new ConditionalCommand(index.run(), Commands.none(), shooter::readyForNote).repeatedly().withName("Attempting to Feed");
    }

    /* CANCEL ALL COMMANDS */
    public Command cancelAll(){
        return new InstantCommand(() -> CommandScheduler.getInstance().cancelAll()).withName("Cancel All");
    }

    private enum State{
        GroundIntake,
        HumanIntake,
        Ejecting,
        AmpEjecting,
        Amp,
        Podium,
        Subwoofer,
        Alliance,
        AutoAim,
        None
    }
}
