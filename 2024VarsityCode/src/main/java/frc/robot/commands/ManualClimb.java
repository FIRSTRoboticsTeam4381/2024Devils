// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pivot;

public class ManualClimb extends Command {
  private CommandPS4Controller controller;
  private Climb climb;
  private Pivot pivot;

  /** Creates a new ManualClimb. */
  public ManualClimb(CommandPS4Controller specialsController, Climb climb, Pivot pivot) {
    controller = specialsController;
    this.climb = climb;
    this.pivot = pivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double positiveJointInput = (controller.getL2Axis()+1.0)/2.0;
    double negativeJointInput = (controller.getR2Axis()+1.0)/2.0;
    double climbInput = controller.getRightY();

    // Deadbands
    positiveJointInput = positiveJointInput<Constants.stickDeadband?0.0:positiveJointInput;
    negativeJointInput = negativeJointInput<Constants.stickDeadband?0.0:negativeJointInput;
    climbInput = climbInput<Constants.stickDeadband?0.0:climbInput;

    /*
    // Disable Control if pivot is too low
    if(pivot.getAngle()<60){
      positiveJointInput = 0.0;
      negativeJointInput = 0.0;
      climbInput = 0.0;
    }
    */

    climb.setBasePercOutput(climbInput);
    climb.setMidPercOutput(positiveJointInput-negativeJointInput);
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
