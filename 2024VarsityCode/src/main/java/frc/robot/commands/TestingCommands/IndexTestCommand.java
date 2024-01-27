// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Indexer;

public class IndexTestCommand extends Command {
  private Indexer s_Index;
  private Trigger intake;
  private Trigger eject;

  private final double indexSpeed = 0.2;

  /** Creates a new IndexTestCommand. */
  public IndexTestCommand(Indexer index, Trigger intakeTrigger, Trigger ejectTrigger) {
    this.s_Index = index;
    this.intake = intakeTrigger;
    this.eject = ejectTrigger;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Index.setSpeed(intake.getAsBoolean()?indexSpeed:eject.getAsBoolean()?-indexSpeed:0);
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
