// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterPivot;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterPivot;

public class ManualPivot extends Command {
  private ShooterPivot s_Pivot;
  private Supplier<Double> leftAxis;

  private double pivotSpeed;

  /** Creates a new ManualPivot. */
  public ManualPivot(ShooterPivot pivot, Supplier<Double> leftAxis) {
    this.s_Pivot = pivot;
    this.leftAxis = leftAxis;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Pivot.rotatePivot(leftAxis.get()*pivotSpeed);
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
