package frc.robot.commands.Pivot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Pivot;

public class PivotCommands {
    private Pivot s_Pivot;

    public PivotCommands(Pivot pivot){
        s_Pivot = pivot;
    }

    public InstantCommand setAmpPos(){
        return new InstantCommand(() -> s_Pivot.setPosition(0 /*TODO find position */), s_Pivot);
    }
    public InstantCommand setDownPos(){
        return new InstantCommand(() -> s_Pivot.setPosition(0), s_Pivot);
    }

    public SequentialCommandGroup goToPosition(InstantCommand positionCommand){
        return new SequentialCommandGroup(positionCommand, new WaitUntilCommand(s_Pivot::atReference));
    }
}
