package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
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
    private Limelight ll;
    private Swerve swerve;

    private State activeState = State.transit;

    // TODO change pivot commands over to profiled motion once that's done

    public ComposedCommands(Intake intake, Index index, Shooter shooter, Pivot pivot, Limelight ll, Swerve swerve){
        this.intake = intake;
        this.index = index;
        this.shooter = shooter;
        this.pivot = pivot;
        this.ll = ll;
        this.swerve = swerve;
    }


    /* INTAKE */

    public Command toggleGroundIntake(){
        return new ConditionalCommand(stopGroundIntake(), groundIntake(), this::groundIntaking);
    }
    private boolean groundIntaking(){
        return activeState == State.groundIntake;
    }
    public Command groundIntake(){
        activeState = State.groundIntake;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                pivot.profiledMove(Pivot.INTAKE_POS),
                new ParallelRaceGroup(
                    intake.run(),
                    index.indexUntilIn(false)
                )
            ),
            stopGroundIntake()
        );
    }
    public Command stopGroundIntake(){
        activeState = State.transit;
        return new ParallelCommandGroup(
            pivot.profiledMove(Pivot.TRANSIT_POS),
            intake.instantStop(),
            index.instantStop()  
        );
    }


    /* HUMAN PLAYER INTAKE */

    /*
     * Should also be able to run off of a toggleOnTrue for the same reasons, everything that
     * needs to stop should stop
     */
    public Command toggleHumanIntake(){
        return new ConditionalCommand(stopHumanIntake(), humanIntake(), this::humanIntaking);
    }
    private boolean humanIntaking(){
        return activeState == State.humanIntake;
    }
    public Command humanIntake(){
        activeState = State.humanIntake;
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                pivot.profiledMove(Pivot.HUMAN_POS),
                index.indexUntilIn(true), // Stops when cancelled
                shooter.eject() // Stops when cancelled
            ),
            stopHumanIntake()
        );
    }
    public Command stopHumanIntake(){
        activeState = State.transit;
        return new ParallelCommandGroup(
            pivot.profiledMove(Pivot.TRANSIT_POS),
            index.instantStop(),
            shooter.instantStopAll()
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

    public Command toggleAmpMode(){
        return new ConditionalCommand(stopAmpMode(), ampMode(), this::inAmpMode);
    }
    private boolean inAmpMode(){
        return activeState == State.amp;
    }
    public Command ampMode(){
        activeState = State.amp;
        return new ParallelCommandGroup(
            pivot.profiledMove(Pivot.AMP_POS),
            shooter.ampShoot() // Stops when cancelled
        );
    }
    public Command stopAmpMode(){
        activeState = State.transit;
        return new ParallelCommandGroup(
            pivot.profiledMove(Pivot.TRANSIT_POS),
            shooter.instantStopAll()
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
        return new InstantCommand(() -> CommandScheduler.getInstance().cancelAll());
    }


    /* AUTO AIM */

    public Command toggleAutoAim(){
        return new ConditionalCommand(stopAutoAim(), autoAim(), this::autoAiming);
    }
    public boolean autoAiming(){
        return activeState == State.autoAim;
    }
    public Command autoAim(){
        activeState = State.autoAim;
        return new SequentialCommandGroup(
            pivot.profiledMove(30),
            new AutoAim(shooter, pivot, ll, swerve)
        );
    }
    public Command stopAutoAim(){
        activeState = State.transit;
        return new ParallelCommandGroup(
            pivot.profiledMove(Pivot.TRANSIT_POS),
            shooter.instantStopAll()
        );
    }

    private enum State{
        transit,
        autoAim,
        groundIntake,
        humanIntake,
        amp
    }
}
