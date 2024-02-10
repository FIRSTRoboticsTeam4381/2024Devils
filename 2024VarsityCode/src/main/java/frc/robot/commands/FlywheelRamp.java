
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Idea for this command
 * This command should run the wheels at full speed until they reach a certain tolerance.
 * At that point, the command finishes, which should run end(), setting the speed to the
 * desired speed?
 */

package frc.robot.commands;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class FlywheelRamp extends Command {
  private Shooter s_Shooter;
  private double speed;

  /** Creates a new ShooterRampUp. */
  public FlywheelRamp(Shooter shooter, double targetVelocity) {
    s_Shooter = shooter;
    speed = targetVelocity;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Shooter.setAll(1.0, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Shooter.getVelocity()>=speed;
  }
}