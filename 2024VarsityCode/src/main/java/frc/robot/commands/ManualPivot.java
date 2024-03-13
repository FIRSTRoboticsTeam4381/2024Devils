// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pivot;

public class ManualPivot extends Command {
  private Supplier<Double> joystick;
  private Pivot pivot;

  /** Creates a new JoystickControl. */
  public ManualPivot(Supplier<Double> joystickAxis, Pivot pivot) {
    this.joystick = joystickAxis;
    this.pivot = pivot;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double axis = joystick.get();
    axis = Math.abs(axis)<Constants.stickDeadband ? 0.0 : axis;
    axis *= -0.4;

    SmartDashboard.putNumber("Pivot Axis", axis);

    // Manual pivot limits
    if(axis>0.0 && (pivot.getAngle()>115&&pivot.getAngle()<330)) {axis = 0.0;}
    if(axis<0.0 && (pivot.getAngle()<=6||pivot.getAngle()>330)) {axis = 0.0;}

    pivot.manualControl(axis+basicFeedforward());
  }

  private double basicFeedforward(){
    final double ffSpeed = 0.015;
    return Math.cos(pivot.getAngle()*(Math.PI/180.0))*ffSpeed;
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
