// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkUtilities.SparkUtilities;
import frc.robot.Constants;

public class Climb extends SubsystemBase {

  /* ATTRIBUTES */
  private CANSparkMax rightBaseMotor;
  private CANSparkMax leftBaseMotor;

  private SparkPIDController baseController;

  private RelativeEncoder baseEncoder;

  /* CONSTRUCTOR */

  /** Creates a new Climb. */
  public Climb() {
    // Motor Setup
    rightBaseMotor = new CANSparkMax(Constants.Climb.rightClimbCAN, MotorType.kBrushless);
    leftBaseMotor = new CANSparkMax(Constants.Climb.leftClimbCAN, MotorType.kBrushless);

    leftBaseMotor.follow(rightBaseMotor, true);

    SparkUtilities.optimizeFrames(rightBaseMotor, true, false, true, false, false, false);
    SparkUtilities.optimizeFrames(leftBaseMotor, false, false, false, false, false, false);

    // Encoder Setup
    baseEncoder = rightBaseMotor.getEncoder();

    baseController = rightBaseMotor.getPIDController();

    // PID Setup
    // TODO Base controller configuration
    baseController.setP(0);
    baseController.setI(0);
    baseController.setD(0);
    baseController.setFF(0);
    baseController.setOutputRange(-1, 1);
  }

  public void setBasePercOutput(double speed){
    rightBaseMotor.set(speed);
  }

  public void setBaseReference(double position){
    baseController.setReference(position, ControlType.kPosition);
  }

  public double getBasePosition(){
    return rightBaseMotor.getEncoder().getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("climb/Base Pivot Position", baseEncoder.getPosition());

    SmartDashboard.putNumber("climb/Left Base Current", leftBaseMotor.getOutputCurrent());
    SmartDashboard.putNumber("climb/Right Base Current", rightBaseMotor.getOutputCurrent());
  }


  public void burnFlash(){
    try{
      Thread.sleep(1000);
      rightBaseMotor.burnFlash();
      Thread.sleep(1000);
      leftBaseMotor.burnFlash();
      Thread.sleep(1000);
    }catch(InterruptedException e){
      DriverStation.reportError("Thread was interrupted while flashing climb", e.getStackTrace());
    }
  }
}
