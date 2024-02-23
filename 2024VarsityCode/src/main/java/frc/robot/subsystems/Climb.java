// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkUtilities;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

  /* ATTRIBUTES */
  private CANSparkMax rightBaseMotor;
  private CANSparkMax leftBaseMotor;
  private CANSparkMax midMotor;

  private SparkPIDController baseController;
  private SparkPIDController middleController;

  private RelativeEncoder baseEncoder;
  private RelativeEncoder midEncoder;

  /* CONSTRUCTOR */

  /** Creates a new Climb. */
  public Climb() {
    // Motor Setup
    rightBaseMotor = new CANSparkMax(Constants.Climb.rightClimbCAN, MotorType.kBrushless);
    leftBaseMotor = new CANSparkMax(Constants.Climb.leftClimbCAN, MotorType.kBrushless);
    midMotor = new CANSparkMax(Constants.Climb.midClimbCAN, MotorType.kBrushless);

    leftBaseMotor.follow(rightBaseMotor, true);

    SparkUtilities.optimizeFrames(rightBaseMotor, true, false, true, false, false, false);
    SparkUtilities.optimizeFrames(leftBaseMotor, false, false, false, false, false, false);
    SparkUtilities.optimizeFrames(midMotor, false, false, true, false, false, false);

    // Encoder Setup
    baseEncoder = rightBaseMotor.getEncoder();
    midEncoder = midMotor.getEncoder();

    baseController = rightBaseMotor.getPIDController();
    middleController = midMotor.getPIDController();

    // PID Setup
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

  public void setBasePercOutput(double speed){
    rightBaseMotor.set(speed);
  }
  public void setMidPercOutput(double speed){
    midMotor.set(speed);
  }

  public void setBaseReference(double position){
    baseController.setReference(position, ControlType.kPosition);
  }
  public void setMiddleReference(double position){
    middleController.setReference(position, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Base Pivot Position", baseEncoder.getPosition());
    SmartDashboard.putNumber("Mid Pivot Position", midEncoder.getPosition());
  }
}
