// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkFlex topFront;
  private CANSparkFlex topBack;
  private CANSparkFlex bottomFront;
  private CANSparkFlex bottomBack;

  private RelativeEncoder shootEncoder;

  private double setPoint = 0.0;

  private Mode mode = Mode.SHOOT;

  public static final int maxRPM = 5000;

  /** Creates a new Shooter. */
  public Shooter() {
    topFront = new CANSparkFlex(Constants.Shooter.topFrontCAN, MotorType.kBrushless);
    topBack = new CANSparkFlex(Constants.Shooter.topBackCAN, MotorType.kBrushless);
    bottomFront = new CANSparkFlex(Constants.Shooter.bottomFrontCAN, MotorType.kBrushless);
    bottomBack = new CANSparkFlex(Constants.Shooter.bottomBackCAN, MotorType.kBrushless);

    topFront.setIdleMode(IdleMode.kCoast);
    topBack.setIdleMode(IdleMode.kCoast);
    bottomFront.setIdleMode(IdleMode.kCoast);
    bottomBack.setIdleMode(IdleMode.kCoast);

    topBack.follow(bottomBack, true);
    bottomFront.follow(bottomBack, true);
    topFront.follow(bottomBack, false);

    shootEncoder = bottomBack.getEncoder();
  }

  public void setMode(Mode mode){
    this.mode = mode;
    topFront.setInverted(mode==Mode.SHOOT||mode==Mode.EJECT_BOTTOM?false:true);
    bottomFront.setInverted(mode==Mode.SHOOT||mode==Mode.EJECT_TOP?true:false);
  }
  public void setSpeed(double speed){
    setPoint = speed;
  }
  public double getTestSpeed(){
    return (mode==Mode.SHOOT?1.0:0.35);
  }
  public double getVelocity(){
    return shootEncoder.getVelocity();
  }
  public boolean isWithinError(double error){
    return (Math.abs(getVelocity()-maxRPM)<error);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    bottomBack.set(setPoint);
  }

  public enum Mode{
    SHOOT,
    EJECT_TOP,
    EJECT_BOTTOM
  }
}
