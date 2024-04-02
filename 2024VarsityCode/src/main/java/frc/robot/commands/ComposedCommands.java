package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class ComposedCommands {
    private Intake intake;
    private Index index;
    private Shooter shooter;
    private Pivot pivot;
    private Limelight ll;
    private Swerve swerve;
    //private LEDs leds;
    private CommandPS4Controller controller;
    private State state = State.None;

    // TODO change pivot commands over to profiled motion once that's done

    public ComposedCommands(CommandPS4Controller controller, Intake intake, Index index, Shooter shooter, Pivot pivot, Limelight ll, Swerve swerve){
        this.intake = intake;
        this.index = index;
        this.shooter = shooter;
        this.pivot = pivot;
        this.ll = ll;
        this.swerve = swerve;
        //this.leds=leds;
        this.controller = controller;
    }

    public Command setRobotState(State state){
        return new InstantCommand(()->this.state=state);
    }


    /* INTAKE */
    public Command groundIntake(Command handoff){
        return new SequentialCommandGroup(
            new ParallelRaceGroup(
                intake.run(),
                new ParallelCommandGroup( // Just get the note into the index
                    pivot.goToAngle(Pivot.Positions.intake, 1),
                    index.indexUntilIn(false)
                )
            ),
            
            new ParallelRaceGroup( // Get note up to the shooter wheels
                handoff,
                index.indexUntilReady(false)
            )
        );
    }

    public Command humanIntake(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                //leds.intakeWaiting(),
                pivot.goToAngle(Pivot.Positions.human, 0),
                index.indexUntilIn(true), // Stops when cancelled
                shooter.eject() // Stops when cancelled
            ),
            new ParallelCommandGroup(
                pivot.goToAngle(Pivot.Positions.transit, 0),
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
    /*
    public Command ampMode(){
        return new ParallelCommandGroup(
            //leds.shooterStatus(),
            pivot.goToAngle(Pivot.Positions.amp, 0),
            shooter.ampShoot() // Stops when cancelled
        ).withName("Amp Mode");
    }
    */
    public Command ampMode(){
        return new ConditionalCommand(
            new ParallelCommandGroup(
                pivot.goToAngle(Pivot.Positions.amp, 0),
                shooter.ampShoot(),
                setRobotState(State.Amp)
            ),
            new ParallelCommandGroup(
                shooter.instantStopAll(),
                pivot.goToAngle(0, 0),
                setRobotState(State.None)
            ),
            ()->{return state!=State.Amp;}
        );
    }

    /* PODIUM MODE */
    /*
    public Command podiumMode(){
        return new ParallelCommandGroup(
            pivot.goToAngle(32, 1),
            shooter.shoot(4670)
        ).withName("Podium Mode");
    }
    */
    public Command podiumMode(){
        return new ConditionalCommand(
            new ParallelCommandGroup(
                pivot.goToAngle(32, 1),
                shooter.shoot(4670),
                setRobotState(State.Podium)
            ), 
            new ParallelCommandGroup(
                pivot.goToAngle(0, 0),
                shooter.instantStopAll(),
                setRobotState(State.None)
            ), 
            ()->{return state!=State.Podium;}
        );
    }

    /* SUBWOOFER MODE */
    /*
    public Command subwooferMode(){
        return new ParallelCommandGroup(
            pivot.goToAngle(46.5, 1),
            shooter.shoot(4000)
        ).withName("Subwoofer Mode");
    }
    */
    public Command subwooferMode(){
        return new ConditionalCommand(
            new ParallelCommandGroup(
                pivot.goToAngle(46.5, 1),
                shooter.shoot(4000),
                setRobotState(State.Subwoofer)
            ), 
            new ParallelCommandGroup(
                pivot.goToAngle(0, 0),
                shooter.instantStopAll(),
                setRobotState(State.None)
            ), 
            ()->{return state!=State.Subwoofer;}
        );
    }

    /* ALLIANCE LINE MODE */
    /*
    public Command allianceLineMode(){
        return new ParallelCommandGroup(
            pivot.goToAngle(35,1),
            shooter.shoot(4475)
        ).withName("Alliance Line Mode");
    }
    */
    public Command allianceLineMode(){
        return new ConditionalCommand(
            new ParallelCommandGroup(
                pivot.goToAngle(35, 1),
                shooter.shoot(4475),
                setRobotState(State.Alliance)
            ), 
            new ParallelCommandGroup(
                pivot.goToAngle(0, 0),
                shooter.instantStopAll(),
                setRobotState(State.None)
            ), 
            ()->{return state!=State.Alliance;}
        );
    }
    

    /* START SHOOTER */
    public Command startShooter(){
        return new ParallelCommandGroup(
            //leds.shooterStatus(),
            shooter.shoot(5000)
        );
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

    /* TOGGLE AUTO AIMING */
    public Command autoAim(boolean stopOnEnd){
        return new SequentialCommandGroup(
            new AutoShooter(pivot, shooter, ll, swerve, stopOnEnd)
        ).withName("Auto Aim");
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
