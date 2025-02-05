// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SparkPosition;

public class Climb extends SubsystemBase {
  // TODO get climbing positions

  /* ATTRIBUTES */
  private SparkMax rightMotor;
  private SparkMax leftMotor;

  private static final SparkMaxConfig LEFT_MOTOR_CONFIG = new SparkMaxConfig();
  private static final SparkMaxConfig RIGHT_MOTOR_CONFIG = new SparkMaxConfig();

  private SparkAbsoluteEncoder absoluteEncoder;

  /* CONSTRUCTOR */

  /** Creates a new Climb. */
  public Climb() {
    leftMotor = new SparkMax(Constants.Climb.leftClimbCAN, MotorType.kBrushless); // Leader
    rightMotor = new SparkMax(Constants.Climb.rightClimbCAN, MotorType.kBrushless); // Follower

    LEFT_MOTOR_CONFIG
		.inverted(true)
		.idleMode(IdleMode.kBrake)
		.smartCurrentLimit(60);

	LEFT_MOTOR_CONFIG.closedLoop
		.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
		.p(3.0)
		.i(0.002)
		.d(0.0)
		.velocityFF(0.0)
		.outputRange(-1.0, 1.0);

	RIGHT_MOTOR_CONFIG
		.idleMode(IdleMode.kBrake)
		.smartCurrentLimit(60)
		.follow(leftMotor.getDeviceId(), true);

    
    leftMotor.configure(LEFT_MOTOR_CONFIG, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);
    rightMotor.configure(RIGHT_MOTOR_CONFIG, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);
    

    absoluteEncoder = leftMotor.getAbsoluteEncoder();
  }

  public void manualControl(double speed){
    //if(getPosition()<=0 && speed < 0.0) {speed = 0;}
    
    leftMotor.set(speed);
  }

  public void setReference(double position){
    leftMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
  }

  public double getAbsolutePosition(){
    return absoluteEncoder.getPosition();
  }

  public Command goToPosition(double position, ClosedLoopSlot slot){
    return new SparkPosition(leftMotor, position, slot, 0.05, this, this::getAbsolutePosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("climb/Absolute Position", getAbsolutePosition());

    SmartDashboard.putNumber("climb/Left Base Current", rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("climb/Right Base Current", leftMotor.getOutputCurrent());
    SmartDashboard.putString("climb/Active Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());
    SmartDashboard.putNumber("climb/I Accum", leftMotor.getClosedLoopController().getIAccum());
  }

}
