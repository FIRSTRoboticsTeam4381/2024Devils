// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {
  private CANSparkMax rightPivot;
  private CANSparkMax leftPivot;

  private double pivotSpeed = 0.0;

  private RelativeEncoder pivotEncoder;
  //private SparkAbsoluteEncoder absoluteEncoder;

  /** Creates a new ShooterPivot. */
  public ShooterPivot() {
    rightPivot = new CANSparkMax(Constants.Pivot.rightPivotCAN, MotorType.kBrushless);
    leftPivot = new CANSparkMax(Constants.Pivot.leftPivotCAN, MotorType.kBrushless);

    leftPivot.follow(rightPivot);

    pivotEncoder = rightPivot.getEncoder();
    //absoluteEncoder = rightPivot.getAbsoluteEncoder(Type.kDutyCycle);
  }

  public void setPivotSpeed(double speed){
    pivotSpeed = speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rightPivot.set(pivotSpeed);

    SmartDashboard.putNumber("Pivot Relative Position", pivotEncoder.getPosition());
    //SmartDashboard.putNumber("Pivot Absolute Position", absoluteEncoder.getPosition());
  }
}
