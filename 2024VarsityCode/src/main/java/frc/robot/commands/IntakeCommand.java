// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    private Intake intake;
    private Trigger intakeButton;
    private Trigger ejectButton;

    private double intakeSpeed = 0.5;

    /** Creates a new IntakeCommand. */
    public IntakeCommand(Intake intake, Trigger intakeButton, Trigger ejectButton) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.intakeButton = intakeButton;
        this.ejectButton = ejectButton;

        addRequirements(intake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intake.setIntakeSpeed(
            intakeButton.getAsBoolean() ? intakeSpeed : 
            ejectButton.getAsBoolean() ? -intakeSpeed : 
            0
        );
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
