// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake;

public class IntakeDefault extends Command {
  private Intake s_Intake;
  private Trigger intakeButton;
  private Trigger ejectButton;

  private static final double INTAKE_SPEED = 0.5;

  /** Creates a new IntakeDefault. */
  public IntakeDefault(Intake s_Intake, Trigger intakeButton, Trigger ejectButton) {
    this.s_Intake = s_Intake;
    this.intakeButton = intakeButton;
    this.ejectButton = ejectButton;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intakeButton.getAsBoolean()){
      s_Intake.setIntakeSpeed(INTAKE_SPEED);
    }else{
      s_Intake.setIntakeSpeed(0);
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
