// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

/*
 * Command that allows simple manual testing of the climb system
 */
public class ClimbTester extends Command {
  private Climb s_Climb;
  private Supplier<Double> basePivotInput;
  private Supplier<Double> midPivotInput;

  /** Creates a new ClimbDefault. */
  public ClimbTester(Climb climb, Supplier<Double> basePivotInput, Supplier<Double> midPivotInput) {
    s_Climb = climb;
    this.basePivotInput = basePivotInput;
    this.midPivotInput = midPivotInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double baseInput = basePivotInput.get();
    double midInput = midPivotInput.get();

    baseInput = Math.abs(baseInput) < Constants.stickDeadband ? 0 : baseInput;
    midInput = Math.abs(midInput) < Constants.stickDeadband ? 0 : midInput;

    // CHANGE Good speeds to run these at
    baseInput *= 0.75;
    midInput *= 0.75;

    if(baseInput!=0.0){
      s_Climb.setBasePercOutput(baseInput);
    }

    if(midInput!=0.0){
      s_Climb.setMidPercOutput(midInput);
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
