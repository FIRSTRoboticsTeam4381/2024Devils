// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

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
  // TODO get climbing positions

  /* ATTRIBUTES */
  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;

  private SparkPIDController climbController;

  private RelativeEncoder climbEncoder;

  /* CONSTRUCTOR */

  /** Creates a new Climb. */
  public Climb() {
    // Motor Setup
    rightMotor = new CANSparkMax(Constants.Climb.rightClimbCAN, MotorType.kBrushless);
    leftMotor = new CANSparkMax(Constants.Climb.leftClimbCAN, MotorType.kBrushless);

    leftMotor.follow(rightMotor, true);

    SparkUtilities.optimizeFrames(rightMotor, true, false, true, false, false, false);
    SparkUtilities.optimizeFrames(leftMotor, false, false, false, false, false, false);

    // Encoder Setup
    climbEncoder = rightMotor.getEncoder();

    climbController = rightMotor.getPIDController();

    // PID Setup
    // TODO Base controller configuration
    climbController.setP(0);
    climbController.setI(0);
    climbController.setD(0);
    climbController.setFF(0);
    climbController.setOutputRange(-1, 1);
  }

  public void setBasePercOutput(double speed){
    rightMotor.set(speed);
  }

  public void setBaseReference(double position){
    climbController.setReference(position, ControlType.kPosition);
  }

  public double getBasePosition(){
    return climbEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("climb/Base Pivot Position", climbEncoder.getPosition());

    SmartDashboard.putNumber("climb/Left Base Current", rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("climb/Right Base Current", leftMotor.getOutputCurrent());
    SmartDashboard.putString("climb/Active Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());
  }


  public void burnFlash(){
    try{
      Thread.sleep(1000);
      rightMotor.burnFlash();
      Thread.sleep(1000);
      leftMotor.burnFlash();
      Thread.sleep(1000);
    }catch(InterruptedException e){
      DriverStation.reportError("Thread was interrupted while flashing climb", e.getStackTrace());
    }
  }
}
