package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
    private Intake s_Intake;

    private static final double INTAKE_SPEED = 0.5;
    
    public IntakeCommands(Intake intake){
        s_Intake = intake;
    }

    public InstantCommand startIntake(){
        return new InstantCommand(() -> s_Intake.setIntakeSpeed(INTAKE_SPEED));
    }
    public InstantCommand stopIntake(){
        return new InstantCommand(() -> s_Intake.setIntakeSpeed(0));
    }
}
