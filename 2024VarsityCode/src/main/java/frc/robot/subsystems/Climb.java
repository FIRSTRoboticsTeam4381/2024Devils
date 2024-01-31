// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private CANSparkMax basePivot;
  private CANSparkMax midPivot;

  private double basePivotSpeed = 0.0;
  private double midPivotSpeed = 0.0;

  /** Creates a new Climb. */
  public Climb() {
    basePivot = new CANSparkMax(Constants.Climb.basePivotCAN, MotorType.kBrushless);
    midPivot = new CANSparkMax(Constants.Climb.midPivotCAN, MotorType.kBrushless);

    basePivot.setIdleMode(IdleMode.kBrake);
    midPivot.setIdleMode(IdleMode.kBrake);
  }

  public void setBaseSpeed(double speed){
    basePivotSpeed = speed;
  }
  public void setMidSpeed(double speed){
    midPivotSpeed = speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    basePivot.set(basePivotSpeed);
    midPivot.set(midPivotSpeed);
  }
}
