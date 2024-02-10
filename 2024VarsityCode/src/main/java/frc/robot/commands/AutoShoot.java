// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends Command {
  private Shooter shooter;
  private Pivot pivot;

  /** Creates a new AutoShoot. */
  public AutoShoot(Shooter shooter, Pivot pivot) {
    this.shooter = shooter;
    this.pivot = pivot;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooting(true);
  }

  private double calculateVelocity(){ //TODO
    return 0.0;
  }
  private double calculateAngle(){ // TODO
    return 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double expectedVelocity = calculateVelocity();
    double expectedAngle = calculateAngle();
    
    shooter.setAll(expectedVelocity/Shooter.maxRPM, true);
    pivot.setPosition(expectedAngle); // TODO conversion between angle and REV position
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setShooting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
