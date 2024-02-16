// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterPivot;

public class ShooterPivotDefault extends Command {
  private ShooterPivot s_Pivot;
  private Supplier<Double> positive;
  private Supplier<Double> negative;

  /** Creates a new ShooterPivotDefault. */
  public ShooterPivotDefault(ShooterPivot pivot, Supplier<Double> positiveInput, Supplier<Double> negativeInput) {
    s_Pivot = pivot;
    this.positive = positiveInput;
    this.negative = negativeInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pivotInput = positive.get()-negative.get();
    
    pivotInput = Math.abs(pivotInput) < Constants.stickDeadband ? 0 : pivotInput;
    
    // CHANGE reversed input and got a better speed
    s_Pivot.setPivotSpeed(-pivotInput*0.5);
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