// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private CANSparkMax rightPivot;
  private CANSparkMax leftPivot;
  private CANSparkFlex midPivot;

  private double basePivotSpeed = 0.0;
  private double midPivotSpeed = 0.0;

  private RelativeEncoder baseEncoder;
  private RelativeEncoder midEncoder;

  /** Creates a new Climb. */
  public Climb() {
    rightPivot = new CANSparkMax(Constants.Climb.rightClimbCAN, MotorType.kBrushless);
    leftPivot = new CANSparkMax(Constants.Climb.leftClimbCAN, MotorType.kBrushless);
    midPivot = new CANSparkFlex(Constants.Climb.midClimbCAN, MotorType.kBrushless);

    rightPivot.setIdleMode(IdleMode.kBrake);
    leftPivot.setIdleMode(IdleMode.kBrake);
    midPivot.setIdleMode(IdleMode.kBrake);

    leftPivot.follow(rightPivot, true);

    baseEncoder = rightPivot.getEncoder();
    midEncoder = midPivot.getEncoder();

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
    rightPivot.set(basePivotSpeed);
    midPivot.set(midPivotSpeed);

    SmartDashboard.putNumber("Base Pivot Position", baseEncoder.getPosition());
    SmartDashboard.putNumber("Mid Pivot Position", midEncoder.getPosition());
  }
}
