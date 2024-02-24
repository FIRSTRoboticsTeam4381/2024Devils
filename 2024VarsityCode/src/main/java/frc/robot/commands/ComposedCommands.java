package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
    private Limelight ll;
    private Swerve swerve;

    // TODO change pivot commands over to profiled motion once that's done

    public ComposedCommands(Intake intake, Index index, Shooter shooter, Limelight ll, Swerve swerve){
        this.intake = intake;
        this.index = index;
        this.shooter = shooter;
        this.ll = ll;
        this.swerve = swerve;
    }


    /* INTAKE */

    /*
     * Should be able to run off of a toggleOnTrue because this will be cancelled by either other commands
     * or the toggle, which would stop the intake running as well as the index.
     */
    public Command groundIntake(){
        return new ParallelCommandGroup(
            //pivot.goToIntake(),
            //pivot.profiledMove(Pivot.INTAKE_POS),
            new ParallelRaceGroup(
                intake.run(),
                index.indexUntilIn(false)
            )
        );
    }

    /*
     * Should also be able to run off of a toggleOnTrue for the same reasons, everything that
     * needs to stop should stop
     */
    public Command humanIntake(){
        return new ParallelCommandGroup(
            //pivot.goToHuman(),
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
            //pivot.profiledMove(Pivot.AMP_POS),
            shooter.ampShoot() // Stops when cancelled
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
        return new ParallelCommandGroup(
            //new InstantCommand(() -> pivot.setAngleReference(pivot.getAngle(), 0)),
            shooter.instantStopAll(),
            index.instantStop(),
            intake.instantStop()
        );
    }

    private boolean autoAiming = false;
    /* TOGGLE AUTO AIMING */
    public Command toggleAutoAim(){
        return new ConditionalCommand(stopAutoAim(), autoAim(), () -> {return autoAiming;});
    }
    public Command autoAim(){
        autoAiming = true;
        return new AutoAim(shooter, ll, swerve);
    }
    public Command stopAutoAim(){
        autoAiming = false;
        return new ParallelCommandGroup(
            //pivot.goToTransit(),
            shooter.instantStopAll()
        );
    }
}
