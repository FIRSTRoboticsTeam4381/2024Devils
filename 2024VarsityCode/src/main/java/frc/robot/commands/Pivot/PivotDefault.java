// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pivot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

public class PivotDefault extends Command {
  private Pivot s_Pivot;
  private Supplier<Double> controllerAxis;
  
  /** Creates a new PivotDefault. */
  public PivotDefault(Pivot pivot, Supplier<Double> axis) {
    s_Pivot = pivot;
    controllerAxis = axis;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double axis = controllerAxis.get();
    axis = Math.abs(axis) < Constants.stickDeadband ? 0 : axis;
    
    s_Pivot.moveSetpoint(axis);
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
