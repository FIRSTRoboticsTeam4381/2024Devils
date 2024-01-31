// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desired Behavior:
 * I want this to mostly rely on automatic position control, but the specialist
 * can still have manual control.
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private CANSparkFlex rightPivot;
  private CANSparkFlex leftPivot;
  private RelativeEncoder pivotEncoder;
  private SparkPIDController pivotController;

  private double setPoint = 0.0;
  private static final double MOVE_SPEED = 0.5;
  private static final double TOLERANCE = 0.1;

  /** Creates a new Pivot. */
  public Pivot() {
    rightPivot = new CANSparkFlex(Constants.Pivot.rightPivotCAN, MotorType.kBrushless);
    leftPivot = new CANSparkFlex(Constants.Pivot.leftPivotCAN, MotorType.kBrushless);

    leftPivot.follow(rightPivot);

    leftPivot.setIdleMode(IdleMode.kBrake);
    rightPivot.setIdleMode(IdleMode.kBrake);

    pivotEncoder = rightPivot.getEncoder();
    pivotController = rightPivot.getPIDController();

    pivotController.setFF(0);
    pivotController.setP(0);
    pivotController.setI(0);
    pivotController.setD(0);
    pivotController.setOutputRange(-1, 1);
  }

  public double getPosition(){
    return pivotEncoder.getPosition();
  }
  public double getAngle(){
    return getPosition()/125*360; // TODO check this, idk what this should be
  }

  public void setPosition(double position){
    setPoint = position;
  }
  public void setAngle(double angle){
    setPoint = angle/360*125; // TODO check this, idk what this should be
  }

  public void moveSetpoint(double movement){
    setPoint += movement*MOVE_SPEED;
  }
  public boolean atReference(){
    return Math.abs(pivotEncoder.getPosition()-setPoint) < TOLERANCE;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pivotController.setReference(setPoint, ControlType.kPosition);
  }
}
