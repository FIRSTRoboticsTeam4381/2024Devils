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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.util.SparkUtilities;
import frc.robot.Constants;
import frc.robot.commands.SparkMaxPosition;

public class Pivot extends SubsystemBase {

  /* ATTRIBUTES */
  
  private CANSparkFlex rightPivot;
  private CANSparkFlex leftPivot;

  private AbsoluteEncoder pivotEncoder;

  private SparkPIDController pivotController;

  // Tested Positions
  public static final double INTAKE_POS = 60;
  public static final double HUMAN_POS = 60; // TODO
  public static final double AMP_POS = 90;


  /* CONSTRUCTORS */

  /** Creates a new Pivot. */
  public Pivot() {
    // Motor Setup
    rightPivot = new CANSparkFlex(Constants.Pivot.rightPivotCAN, MotorType.kBrushless);
    leftPivot = new CANSparkFlex(Constants.Pivot.leftPivotCAN, MotorType.kBrushless);

    rightPivot.follow(leftPivot);
    leftPivot.setInverted(true);

    leftPivot.setIdleMode(IdleMode.kBrake);
    rightPivot.setIdleMode(IdleMode.kBrake);

    SparkUtilities.optimizeFrames(rightPivot, true, false, true, false, false, true);
    SparkUtilities.optimizeFrames(leftPivot, false, false, false, false, false, false);

    // Encoder Setup
    pivotEncoder = leftPivot.getAbsoluteEncoder(Type.kDutyCycle);

    // PID Setup
    pivotController = leftPivot.getPIDController();
    pivotController.setFeedbackDevice(pivotEncoder);
    pivotController.setOutputRange(-1, 1);
    pivotController.setP(0.01);
    pivotController.setI(0.0);
    pivotController.setD(0.0);
    pivotController.setFF(0.0);
  }


  /* METHODS */
  /*
   * The way I want these to work is setSpeed will only be called by manual control, and
   * set position will only be called by auto control, and the single set positions (see
   * COMMANDS) should only be called by instant commands by button presses.
   */
  public void setPercOutput(double speed){
    rightPivot.set(speed);
  }
  public void setDesiredAngle(double position){
    pivotController.setReference(position, ControlType.kPosition);
  }

  public double getCurrentAngle(){
    return pivotEncoder.getPosition();
  }


  /* COMMANDS */

  // Set position commands
  public Command goTo(double position){
    return new SparkMaxPosition(rightPivot, position, 0, 10, this);
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Pivot Absolute Angle", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Speed", leftPivot.get());
  }
}
