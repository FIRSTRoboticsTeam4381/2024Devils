// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.lib.util.SparkOptimizer;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {
  private CANSparkMax rightPivot;
  private CANSparkMax leftPivot;

  private RelativeEncoder pivotEncoder;
  private SparkAbsoluteEncoder absoluteEncoder;

  private SparkPIDController pivotController;

  private double desiredAngle = 0.0;

  /** Creates a new ShooterPivot. */
  public ShooterPivot() {
    rightPivot = new CANSparkMax(Constants.Pivot.rightPivotCAN, MotorType.kBrushless);
    leftPivot = new CANSparkMax(Constants.Pivot.leftPivotCAN, MotorType.kBrushless);

    rightPivot.follow(leftPivot);

    leftPivot.setIdleMode(IdleMode.kBrake);
    leftPivot.setInverted(true);
    rightPivot.setIdleMode(IdleMode.kBrake);

    pivotEncoder = leftPivot.getEncoder();
    absoluteEncoder = leftPivot.getAbsoluteEncoder(Type.kDutyCycle);

    pivotController = leftPivot.getPIDController();
    pivotController.setFeedbackDevice(absoluteEncoder);
    pivotController.setOutputRange(-1, 1);

    pivotController.setP(0.01);
    pivotController.setI(0.0);
    pivotController.setD(0.0);
    pivotController.setFF(0.0);

    SparkOptimizer.optimizeFrames(leftPivot, true, false, true, false, false, true);
    SparkOptimizer.optimizeFrames(rightPivot, false, false, false, false, false, false);
  }

  public void setPivotSpeed(double speed){
    leftPivot.set(speed);
  }
  public double getPivotSpeed(){
    return leftPivot.get();
  }

  public double getAngle(){
    return absoluteEncoder.getPosition();
  }

  public void setDesiredAngle(double angle){
    desiredAngle = angle;
  }

  public void setPositionReference(double angle){
    pivotController.setReference(angle, ControlType.kPosition);
  }

  public Command goToPosition(double angle){
    return new FunctionalCommand(
      () -> {setPositionReference(angle);}, 
      () -> {}, 
      (interrupted) -> {}, 
      () -> {return false;}, 
      this);
  }

  /* TRAPEZOID MOTION PROFILING */

  private TrapezoidProfile.State getDesiredState(){
    return new TrapezoidProfile.State(desiredAngle, 0);
  }
  private TrapezoidProfile.State getCurrentState(){
    return new TrapezoidProfile.State(absoluteEncoder.getPosition(), absoluteEncoder.getVelocity());
  }
  private void useState(TrapezoidProfile.State state){
    pivotController.setReference(state.position, ControlType.kPosition);
  }

  /*
  public TrapezoidProfileCommand pivotToAngle(double angle){
    return new TrapezoidProfileCommand(
      new TrapezoidProfile(new Constraints(2, 2)),
      (state) -> useState(state), 
      this:getDesiredState, 
      this:getCurrentState, 
      this);
  }
  */

  /* END TRAPEZOID MOTION PROFILE */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Pivot Relative Position", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Absolute Angle", absoluteEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Speed", leftPivot.get());
  }
}
