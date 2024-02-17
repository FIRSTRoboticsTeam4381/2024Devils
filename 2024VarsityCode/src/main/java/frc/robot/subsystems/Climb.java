// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desired Behavior:
 * Eventually, I want this to be able to be moved via both manual control
 * and position control. I'm not entirely sure how to achieve that yet
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

  /* ATTRIBUTES */
  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;
  private CANSparkMax midMotor;

  private SparkPIDController baseController;
  private SparkPIDController middleController;


  /* CONSTRUCTOR */

  /** Creates a new Climb. */
  public Climb() {
    rightMotor = new CANSparkMax(Constants.Climb.rightClimbCAN, MotorType.kBrushless);
    leftMotor = new CANSparkMax(Constants.Climb.leftClimbCAN, MotorType.kBrushless);
    midMotor = new CANSparkMax(Constants.Climb.midClimbCAN, MotorType.kBrushless);

    leftMotor.follow(rightMotor, true);

    baseController = rightMotor.getPIDController();
    middleController = midMotor.getPIDController();

    // TODO Base controller configuration
    baseController.setP(0);
    baseController.setI(0);
    baseController.setD(0);
    baseController.setFF(0);
    baseController.setOutputRange(-1, 1);

    // TODO Middle controller configuration
    middleController.setP(0);
    middleController.setI(0);
    middleController.setD(0);
    middleController.setFF(0);
    middleController.setOutputRange(-1, 1);
  }

  public void setBasePosition(double position){
    baseController.setReference(position, ControlType.kPosition);
  }
  public void setMiddlePosition(double position){
    middleController.setReference(position, ControlType.kPosition);
  }

  /* TODO ONLY FOR TESTING */
  public void setBaseSpeed(double speed){
    rightMotor.set(speed);
  }
  public void setMiddleSpeed(double speed){
    midMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
