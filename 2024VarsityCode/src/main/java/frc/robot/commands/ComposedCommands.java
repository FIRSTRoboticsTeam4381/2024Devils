package frc.robot.commands;

import java.util.concurrent.locks.Condition;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class ComposedCommands {
    private Intake intake;
    private Index index;
    private Shooter shooter;
    private Pivot pivot;
    private Condition condition = Condition.NONE;

    public ComposedCommands(Intake intake, Index index, Shooter shooter, Pivot pivot){
        this.intake = intake;
        this.index = index;
        this.shooter = shooter;
        this.pivot = pivot;
    }

    /* TOGGLE COMMANDS */

    public Command toggleGroundIntake(){
        return new ConditionalCommand(stopIntaking(), intake(), ()->{return condition==Condition.GROUND_INTAKE;});
    }
    public Command toggleHumanIntake(){
        return new ConditionalCommand(stopIntaking(), humanPlayerIntake(), ()->{return condition==Condition.HUMAN_INTAKE;});
    }
    public Command toggleShooter(){
        return new ConditionalCommand(stopAll(), activateShooter(), () -> {return condition==Condition.SHOOTING;});
    }
    public Command toggleAmp(){
        return new ConditionalCommand(new ParallelCommandGroup(stopAll(), new InstantCommand(()->pivot.setDesiredAngle(0.0))), setupAmp(), () -> {return condition==Condition.AMP;});
    }
    public Command toggleAutoAim(){
        return new ConditionalCommand(stopAll(), new AutoAim(shooter, pivot), () -> {return condition==Condition.AUTO_AIM;});
    }

    
    /* COMPOSITIONS */
    public Command humanPlayerIntake(){
        condition = Condition.HUMAN_INTAKE;
        return new SequentialCommandGroup(
            new InstantCommand(() -> pivot.setDesiredAngle(Pivot.HUMAN_POS), pivot),
            new InstantCommand(() -> shooter.setPercOutput(-0.1), shooter),
            index.indexUntilIn(-1),
            new InstantCommand(() -> shooter.stopAll(), shooter),
            stopIntaking()
        );
    }
    public Command intake(){
        condition = Condition.GROUND_INTAKE;
        return new SequentialCommandGroup(
            new InstantCommand(() -> pivot.setDesiredAngle(Pivot.INTAKE_POS), pivot),
            intake.start(),
            index.indexUntilIn(1),
            stopIntaking()
        );
    }
    public Command stopIntaking(){
        condition = Condition.NONE;
        return new ParallelCommandGroup(
            new InstantCommand(() -> pivot.setDesiredAngle(0.0), pivot),
            intake.stop()
        );
    }
    public Command eject(){
        condition = Condition.EJECT;
        return new ParallelCommandGroup(
            intake.eject(),
            index.eject(),
            shooter.eject()
        );
    }

    public Command stopAll(){
        condition = Condition.NONE;
        return new ParallelCommandGroup(
            intake.stop(),
            index.stop(),
            shooter.stopAll()
        );
    }

    // Sets 100% speed just to ramp up to the velocity faster, but then hand off to PID control to maintain that velocity
    public Command activateShooter(){
        condition = Condition.SHOOTING;
        return new SequentialCommandGroup(
            new InstantCommand(() -> CommandScheduler.getInstance().schedule(pivot.getDefaultCommand())), // Set pivot to run default command so that it cancels other commands
            new FlywheelRamp(shooter, 1500, 10), // 1500 seems to work well at most distances
            new InstantCommand(() -> shooter.setVelocity(1500), shooter)
        );
    }

    public Command setupAmp(){
        condition = Condition.AUTO_AIM;
        condition = Condition.AMP;
        return new ParallelCommandGroup(
            shooter.ampShoot(),
            new InstantCommand(() -> pivot.setDesiredAngle(Pivot.AMP_POS), pivot)
        );
    }


    private enum Condition{
        NONE,
        GROUND_INTAKE,
        HUMAN_INTAKE,
        EJECT,
        SHOOTING,
        AMP,
        AUTO_AIM
    }
}
