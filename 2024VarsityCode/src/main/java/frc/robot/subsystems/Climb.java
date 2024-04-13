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

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkUtilities.SparkUtilities;
import frc.robot.Constants;
import frc.robot.commands.SparkPosition;

public class Climb extends SubsystemBase {
  // TODO get climbing positions

  /* ATTRIBUTES */
  private CANSparkMax rightMotor;
  private CANSparkMax leftMotor;

  private CANSparkMax leader;
  private CANSparkMax follower;

  private SparkPIDController climbController;

  private SparkAbsoluteEncoder absoluteEncoder;

  /* CONSTRUCTOR */

  /** Creates a new Climb. */
  public Climb() {
    // Motor Setup
    rightMotor = new CANSparkMax(Constants.Climb.rightClimbCAN, MotorType.kBrushless);
    leftMotor = new CANSparkMax(Constants.Climb.leftClimbCAN, MotorType.kBrushless);
    absoluteEncoder = leftMotor.getAbsoluteEncoder(Type.kDutyCycle);

    leader = leftMotor;
    follower = rightMotor;

    leader.setInverted(true);
    follower.follow(leader, true);

    leader.setIdleMode(IdleMode.kBrake);
    follower.setIdleMode(IdleMode.kBrake);

    leader.setSmartCurrentLimit(60);
    follower.setSmartCurrentLimit(60);

    SparkUtilities.optimizeFrames(leader, true, false, false, false, false, true);
    SparkUtilities.optimizeFrames(follower, false, false, true, false, false, false);


    climbController = leader.getPIDController();
    // PID Setup
    // TODO Base controller configuration
    climbController.setFeedbackDevice(absoluteEncoder);
    climbController.setP(3.0, 0);
    climbController.setI(0.00225, 0);
    climbController.setD(0, 0);
    climbController.setFF(0, 0);
    climbController.setOutputRange(-1, 1);
  }

  public void manualControl(double speed){
    //if(getPosition()<=0 && speed < 0.0) {speed = 0;}
    
    leader.set(speed);
  }

  public void setReference(double position){
    climbController.setReference(position, ControlType.kPosition, 0);
  }

  public double getAbsolutePosition(){
    return absoluteEncoder.getPosition();
  }

  public Command goToPosition(double position, int slot){
    return new SparkPosition(leader, position+0.25, slot, 0.05, this, this::getAbsolutePosition);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("climb/Absolute Position", getAbsolutePosition());

    SmartDashboard.putNumber("climb/Left Base Current", rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("climb/Right Base Current", leftMotor.getOutputCurrent());
    SmartDashboard.putString("climb/Active Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());
    SmartDashboard.putNumber("climb/I Accum", climbController.getIAccum());
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
