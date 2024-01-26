// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
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

  private double setPoint = 0.0;

  private SparkPIDController shooterPID;

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

    shooterPID = bottomBack.getPIDController();
    shooterPID.setFF(0);
    shooterPID.setP(0);
    shooterPID.setI(0);
    shooterPID.setD(0);
    shooterPID.setFeedbackDevice(bottomBack.getEncoder());
    shooterPID.setOutputRange(0, 1);
  }

  public void setMode(Mode mode){
    topFront.setInverted(mode==Mode.SHOOT||mode==Mode.EJECT_BOTTOM?false:true);
    bottomFront.setInverted(mode==Mode.SHOOT||mode==Mode.EJECT_TOP?true:false);
  }
  public void setSpeed(double speed){
    setPoint = speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterPID.setReference(setPoint, ControlType.kDutyCycle);
  }

  public enum Mode{
    SHOOT,
    EJECT_TOP,
    EJECT_BOTTOM
  }
}
