// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pivot;

public class ManualClimb extends Command {
  private Climb climb;
  private Pivot pivot;

  private Supplier<Double> joystick;
  private Trigger button;

  private boolean previousState = false;

  /** Creates a new ManualClimb. */
  public ManualClimb(Supplier<Double> joystick, Trigger button, Climb climb, Pivot pivot) {
    this.climb = climb;
    this.pivot = pivot;
    this.joystick = joystick;
    this.button = button;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(pivot.getAngle() < 60){
      climb.setBasePercOutput(joystick.get());
      if(button.getAsBoolean()){
        if(!previousState){
          togglePosition();
        }
        previousState = true;
      }else{
        previousState = false;
      }
    }else{
      climb.setBasePercOutput(0.0);
      climb.setMidPercOutput(0.0);
    }
  }

  private void togglePosition(){
    if(climb.getMiddleReference() == 0.0){
      climb.setMiddleReference(Climb.MID_READY_POS);
    }else{
      climb.setMiddleReference(0.0);
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
