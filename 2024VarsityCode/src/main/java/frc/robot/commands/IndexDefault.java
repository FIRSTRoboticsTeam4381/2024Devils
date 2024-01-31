// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Indexer;

public class IndexDefault extends Command {
  private Indexer s_Index;
  private Trigger intakeButton;
  private Trigger ejectButton;

  private static final double INDEX_SPEED = 0.5;

  /** Creates a new IndexDefault. */
  public IndexDefault(Indexer index, Trigger intakeButton, Trigger ejectButton) {
    s_Index = index;
    this.intakeButton = intakeButton;
    this.ejectButton = ejectButton;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(index);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeButton.getAsBoolean()){
      s_Index.setSpeed(INDEX_SPEED);
    }else if(ejectButton.getAsBoolean()){
      s_Index.setSpeed(-INDEX_SPEED);
    }else{
      s_Index.setSpeed(0);
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
