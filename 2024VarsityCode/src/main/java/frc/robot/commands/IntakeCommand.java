// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private Intake intake;
    private CommandPS4Controller controller;

    /** Creates a new IntakeCommand. */
    public IntakeCommand(Intake intake, CommandPS4Controller controller) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intake = intake;
        this.controller = controller;
        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(controller.cross().getAsBoolean()){
            intake.setIntakeSpeed(1);
        }else if(controller.circle().getAsBoolean()){
            intake.setIntakeSpeed(-1);
        }else{
            intake.setIntakeSpeed(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
    return false;
    }
}
