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


    /* INTAKE */
/*
    public Command groundIntake(Command handoff){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                //leds.intakeWaiting(),
                pivot.goToAngle(Pivot.Positions.intake),
                new ParallelRaceGroup(
                    intake.run(),
                    index.indexUntilIn(false)
                )
            ), // At this point, we have a note
            new ParallelRaceGroup(
                intake.run(),
                handoff, // Hand off to something else to get pivot moving early
                index.indexUntilReady(false) // Keep index running until note is all the way in
            )
        ).withName("Ground Intake");
    }
    */
    public Command groundIntake(Command handoff){
        return new ParallelRaceGroup(
            intake.run(),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    index.indexUntilIn(false),
                    pivot.goToAngle(Pivot.Positions.intake)
                ),
                new ParallelRaceGroup(
                    index.indexUntilReady(false),
                    handoff
                )
            )
        );
    }

    public Command humanIntake(){
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                //leds.intakeWaiting(),
                pivot.goToAngle(Pivot.Positions.human),
                index.indexUntilIn(true), // Stops when cancelled
                shooter.eject() // Stops when cancelled
            ),
            new ParallelCommandGroup(
                pivot.goToAngle(Pivot.Positions.transit),
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
            //leds.shooterStatus(),
            pivot.goToAngle(Pivot.Positions.amp),
            shooter.ampShoot() // Stops when cancelled
        ).withName("Amp Mode");
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
    public Command autoAim(){
        return new SequentialCommandGroup(
            new AutoShooter(pivot, shooter, ll, swerve)
        ).withName("Auto Aim");
    }
}
