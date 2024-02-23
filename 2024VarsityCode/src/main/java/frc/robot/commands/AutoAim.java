// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoAim extends Command {
  private Shooter shooter;
  private Pivot pivot;
  private Swerve swerve;
  private Limelight ll;

  private double currentVelocity = 0.0;
  private double currentAngle = 30.0;

  /** Creates a new AutoAim. */
  public AutoAim(Shooter shooter, Pivot pivot, Limelight ll) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.ll = ll;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private void calcAngle(){
    if(ll.hasTargets() == 1) currentAngle = 49.62307316*Math.pow(ll.distanceFromGoal(), -0.425990916); // r^2 = 0.995
    if(currentAngle > 90) currentAngle = 90;
  }
  private void calcVelocity(){
    currentVelocity = 1700;
    //if(ll.hasTargets() == 1) currentVelocity = ll.distanceFromGoal() <= 2.5 ? 1000 : 1500;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calcAngle();
    calcVelocity();

    SmartDashboard.putNumber("autoaim/Calculated Velocity", currentVelocity);
    SmartDashboard.putNumber("autoaim/Calculated Angle", currentAngle);

    pivot.setAngleReference(currentAngle, 1);
    shooter.setVelocity(currentVelocity, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //CommandScheduler.getInstance().schedule(pivot.goToTransit().alongWith(shooter.instantStopAll()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
