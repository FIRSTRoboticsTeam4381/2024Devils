// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class AutoAim extends Command {
  private Shooter shooter;
  private Pivot pivot;

  /** Creates a new AutoAim. */
  public AutoAim(Shooter shooter, Pivot pivot) {
    this.shooter = shooter;
    this.pivot = pivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double calcAngle(){
    double angle = 45.45445432*Math.pow(Limelight.getDistanceFromGoal(), -0.6264155683);
    return angle;
  }
  private double calcVelocity(){
    double velocity = Limelight.getDistanceFromGoal() <= 2.5 ? 1000 : 1500;
    return velocity;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = calcAngle();
    double velocity = calcVelocity();

    SmartDashboard.putNumber("autoaim/Calculated Velocity", velocity);
    SmartDashboard.putNumber("autoaim/Calculated Angle", angle);

    //pivot.setAngleReference(angle);
    //shooter.setVelocity(velocity, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().schedule(pivot.goToTransit().alongWith(shooter.instantStopAll()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
