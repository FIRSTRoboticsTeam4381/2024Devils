// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ManualClimb extends Command {
  private CommandPS4Controller controller;
  private Climb climb;

  /** Creates a new ManualClimb. */
  public ManualClimb(CommandPS4Controller specialsController, Climb climb) {
    controller = specialsController;
    this.climb = climb;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double climbInput = -controller.getRightY();

    // Deadbands
    climbInput = Math.abs(climbInput)<Constants.stickDeadband?0.0:climbInput;

    /*
    if(climb.getBasePosition()<=3.0 && climbInput<0.0)
    {
      climbInput = 0;
    }
    */

    climb.setBasePercOutput(climbInput*0.75);
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
