// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;

public class IntakeTestCommand extends Command {
  private Intake s_Intake;
  private Trigger intake;
  private Trigger eject;

  private final double intakeSpeed = 0.4;

  /** Creates a new IntakeTestCommand. */
  public IntakeTestCommand(Intake intake, Trigger intakeTrigger, Trigger ejectTrigger) {
    this.s_Intake = intake;
    this.intake = intakeTrigger;
    this.eject = ejectTrigger;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.setIntakeSpeed(intake.getAsBoolean()?intakeSpeed:eject.getAsBoolean()?-intakeSpeed:0);
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
