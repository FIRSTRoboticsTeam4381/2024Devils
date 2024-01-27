// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterRamp extends Command {
  private Shooter s_Shooter;
  private BangBangController controller;

  private double setSpeed;
  private double error;

  /** Creates a new ShooterRamp. This command rapidly ramps speed up to the set point, and ends when the velocity
   * is within the set error.
  */
  public ShooterRamp(Shooter shooter, double speed, double error) {
    this.s_Shooter = shooter;
    this.setSpeed = speed;
    this.error = error;

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
    s_Shooter.setSpeed(controller.calculate(s_Shooter.getVelocity(), setSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Shooter.isWithinError(error);
  }
}
