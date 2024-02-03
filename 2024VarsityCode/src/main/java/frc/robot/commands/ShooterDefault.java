// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;

public class ShooterDefault extends Command {
  private Shooter s_Shooter;
  private Trigger shootButton;
  private Trigger ejectTopButton;
  private Trigger ejectBottomButton;

  private static final double SHOOT_SPEED = 0.5;
  private static final double EJECT_SPEED = 0.2;

  /** Creates a new ShooterDefault. */
  public ShooterDefault(Shooter shooter, Trigger shoot, Trigger ejectTop, Trigger ejectBottom) {
    s_Shooter = shooter;
    shootButton = shoot;
    ejectTopButton = ejectTop;
    ejectBottomButton = ejectBottom;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shootButton.getAsBoolean()){
      s_Shooter.shoot(SHOOT_SPEED);
    }else if(ejectTopButton.getAsBoolean()){
      s_Shooter.ejectTop(EJECT_SPEED);
    }else if(ejectBottomButton.getAsBoolean()){
      s_Shooter.ejectBottom(EJECT_SPEED);
    }else{
      s_Shooter.stopAll();
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
