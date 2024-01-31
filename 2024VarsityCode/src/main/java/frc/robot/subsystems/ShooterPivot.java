// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {
  private CANSparkFlex rightPivot;
  private CANSparkFlex leftPivot;

  private double pivotSpeed = 0.0;

  /** Creates a new ShooterPivot. */
  public ShooterPivot() {
    rightPivot = new CANSparkFlex(Constants.Shooter.rightPivotCAN, MotorType.kBrushless);
    leftPivot = new CANSparkFlex(Constants.Shooter.leftPivotCAN, MotorType.kBrushless);

    leftPivot.follow(rightPivot);
  }

  public void setPivotSpeed(double speed){
    pivotSpeed = speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightPivot.set(pivotSpeed);
  }
}
