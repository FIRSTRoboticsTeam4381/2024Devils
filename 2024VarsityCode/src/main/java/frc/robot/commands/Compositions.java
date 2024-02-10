package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Pivot;

public class Compositions {
    private Intake intake;
    private Index index;
    private Pivot pivot;

    private boolean intaking = false;
    
    public Compositions(Intake intake, Index index, Pivot pivot){
        this.intake = intake;
        this.index = index;
        this.pivot = pivot;
    }

    private boolean isIntaking(){
        return intaking;
    }
    public ConditionalCommand toggleIntaking(){
        return new ConditionalCommand(stopIntaking(), intakeNote(), this::isIntaking);
    }
    public SequentialCommandGroup intakeNote(){
        return new SequentialCommandGroup(
            pivot.goToIntake(),
            intake.start(),
            index.indexUntilIn(),
            stopIntaking()
        );
    }
    public ParallelCommandGroup stopIntaking(){
        return new ParallelCommandGroup(
            index.stop(),
            intake.stop(),
            pivot.goToBottom()
        );
    }

}
