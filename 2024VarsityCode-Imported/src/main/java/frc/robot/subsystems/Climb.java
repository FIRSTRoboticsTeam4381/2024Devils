// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;

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
  private SparkMax rightMotor;
  private SparkMax leftMotor;

  private SparkClosedLoopController climbController;

  private SparkAbsoluteEncoder absoluteEncoder;

  /* CONSTRUCTOR */

  /** Creates a new Climb. */
  public Climb() {
    climbController = leader.getClosedLoopController();
    climbController.setFeedbackDevice(absoluteEncoder);
    climbController.setP(3.0, 0);
    climbController.setI(0.002, 0);
    climbController.setD(0, 0);
    climbController.setFF(0, 0);
    climbController.setOutputRange(-1, 1);

    // Motor Setup
    SparkBaseConfig leftConfig = new SparkMaxConfig()
                                .inverted(true)
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(60);
    //leftConfig.closedLoop.feedbackSensor()
    SparkBaseConfig rightConfig = new SparkMaxConfig().follow(Constants.Climb.leftClimbCAN, true).idleMode(IdleMode.kBrake).smartCurrentLimit(60);

    leftMotor = new SparkMax(Constants.Climb.leftClimbCAN, MotorType.kBrushless);
    rightMotor = new SparkMax(Constants.Climb.rightClimbCAN, MotorType.kBrushless);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);
    rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters,  PersistMode.kPersistParameters);
    

    absoluteEncoder = leftMotor.getAbsoluteEncoder();
    leader = leftMotor;
    follower = rightMotor;

    //SparkUtilities.optimizeFrames(leader, true, false, false, false, false, true);
    //SparkUtilities.optimizeFrames(follower, false, false, true, false, false, false);


    
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
    return new SparkPosition(leader, position, slot, 0.05, this, this::getAbsolutePosition);
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
