
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Idea for this command
 * This command should run the wheels at full speed until they reach a certain tolerance.
 * At that point, the command finishes, which should run end(), setting the speed to the
 * desired speed?
 */

package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterRampUp extends Command {
  private BangBangController controller;
  private Shooter s_Shooter;
  private double speed;
  private double tolerance;
  private Shooter.Mode mode;

  /** Creates a new ShooterRampUp. */
  public ShooterRampUp(Shooter shooter, double desiredSpeed, double tolerance, Shooter.Mode mode) {
    s_Shooter = shooter;
    speed = desiredSpeed;
    this.tolerance = tolerance;
    this.mode = mode;

    controller = new BangBangController();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Shooter.setAll(controller.calculate(s_Shooter.getVelocity(), speed*Shooter.maxRPM), mode);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Shooter.setAll(speed, mode);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(s_Shooter.getVelocity()-speed*Shooter.maxRPM) < (speed*Shooter.maxRPM*tolerance));
  }
}
