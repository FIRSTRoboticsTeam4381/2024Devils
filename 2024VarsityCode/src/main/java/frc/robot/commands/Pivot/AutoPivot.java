// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Auto calculate proper angle to shoot and set the pivot set point to
 * that angle
 */

package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

public class AutoPivot extends Command {
  private Pivot s_Pivot;

  /** Creates a new AutoPivot. */
  public AutoPivot(Pivot pivot) {
    s_Pivot = pivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distFromGoal = getDistFromGoal();
    double fireAngle = calcAngle(distFromGoal, calcVelocity(distFromGoal));

    s_Pivot.setAngle(fireAngle);
  }

  private double getDistFromGoal(){
    return 0.0; // TODO
  }
  private double calcAngle(double dist, double velocity){
    return 0.0; // TODO
  }
  private double calcVelocity(double dist){
    return 0.0; // TODO
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
