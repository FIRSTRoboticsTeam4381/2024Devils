// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TestingCommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPivot;

public class PivotTestCommand extends Command {
  private ShooterPivot s_Pivot;
  private Supplier<Double> leftAxis;

  /** Creates a new PivotTestCommand. */
  public PivotTestCommand(ShooterPivot pivot, Supplier<Double> leftAxis) {
    this.s_Pivot = pivot;
    this.leftAxis = leftAxis;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Pivot.setPivotSpeed(Math.abs(leftAxis.get()) < Constants.stickDeadband?0:leftAxis.get()*0.2);
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
