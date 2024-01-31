// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {
  private CANSparkMax rightPivot;
  private CANSparkMax leftPivot;
  private RelativeEncoder rightEncoder;
  private RelativeEncoder leftEncoder;
  private SparkAbsoluteEncoder pivotAbsolute;
  private SparkPIDController pivotPID;

  private double pivotSetPoint = 0.0;
  private double pivotSpeed = 0.0; // TODO for testing

  /** Creates a new ShooterPivot. */
  public ShooterPivot() {
    rightPivot = new CANSparkMax(Constants.Shooter.rightPivotCAN, MotorType.kBrushless);
    leftPivot = new CANSparkMax(Constants.Shooter.leftPivotCAN, MotorType.kBrushless);
    rightEncoder = rightPivot.getEncoder();
    leftEncoder = leftPivot.getEncoder();
    pivotAbsolute = rightPivot.getAbsoluteEncoder(Type.kDutyCycle);

    leftPivot.follow(rightPivot);

    pivotPID = rightPivot.getPIDController();
    pivotPID.setFeedbackDevice(rightEncoder);
    pivotPID.setP(0);
    pivotPID.setI(0);
    pivotPID.setD(0);
    pivotPID.setFF(0);
    pivotPID.setOutputRange(-1, 1);
    rightPivot.setIdleMode(IdleMode.kBrake);
  }

  public void setPivotAngle(double angle){
    pivotSetPoint = Conversions.degreesToRev(angle, 1.0/125.0);
  }
  public double getAngleSetpoint(){
    return Conversions.revToDegrees(pivotSetPoint, 1.0/125.0);
  }
  public void rotatePivot(double rotation){
    pivotSetPoint += rotation;
  }

  // Called once per scheduler run
  @Override
  public void periodic(){
    pivotPID.setReference(pivotSetPoint, ControlType.kPosition);
  }
}
