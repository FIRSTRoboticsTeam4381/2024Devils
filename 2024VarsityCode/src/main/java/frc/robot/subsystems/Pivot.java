// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desired Behavior:
 * I want this to mostly rely on automatic position control, but the specialist
 * can still have manual control.
 */

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.commands.SparkMaxPosition;

public class Pivot extends SubsystemBase {

  /* ATTRIBUTES */
  private CANSparkFlex rightPivot;
  private CANSparkFlex leftPivot;

  private RelativeEncoder pivotEncoder;
  private AbsoluteEncoder absoluteEncoder;
  private SparkPIDController pivotController;

  private static final int AMP_POS = 0; // TODO
  private static final int CLIMB_POS = 0; // TODO
  private static final int BOTTOM_POS = 0; // TODO
  private static final int INTAKE_POS = 0; // TODO

  /* CONSTRUCTORS */

  /** Creates a new Pivot. */
  public Pivot() {
    rightPivot = new CANSparkFlex(Constants.Pivot.rightPivotCAN, MotorType.kBrushless);
    leftPivot = new CANSparkFlex(Constants.Pivot.leftPivotCAN, MotorType.kBrushless);

    absoluteEncoder = rightPivot.getAbsoluteEncoder(Type.kDutyCycle);

    leftPivot.follow(rightPivot);

    leftPivot.setIdleMode(IdleMode.kBrake);
    rightPivot.setIdleMode(IdleMode.kBrake);

    pivotEncoder = rightPivot.getEncoder();
    pivotController = rightPivot.getPIDController();

    // TODO configure
    pivotController.setFF(0);
    pivotController.setP(0);
    pivotController.setI(0);
    pivotController.setD(0);
    pivotController.setOutputRange(-1, 1);
  }


  /* METHODS */

  public void setSpeed(double speed){
    rightPivot.set(speed);
  }

  public void setPosition(double position){
    pivotController.setReference(position, ControlType.kPosition);
  }


  /* COMMANDS */

  public Command goTo(double position){
    return new SparkMaxPosition(rightPivot, position, 0, /*TODO*/10, this);
  }

  public Command goToAmp(){
    return goTo(AMP_POS);
  }

  public Command goToClimb(){
    return goTo(CLIMB_POS);
  }

  public Command goToBottom(){
    return goTo(BOTTOM_POS);
  }

  public Command goToIntake(){
    return goTo(INTAKE_POS);
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
