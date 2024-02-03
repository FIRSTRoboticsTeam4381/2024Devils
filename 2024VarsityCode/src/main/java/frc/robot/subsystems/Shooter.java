// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkFlex propMotor;
  private CANSparkFlex topMotor;
  private CANSparkFlex bottomMotor;

  private double propSpeed = 0.0;
  private double topSpeed = 0.0;
  private double bottomSpeed = 0.0;

  private RelativeEncoder propEncoder;
  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;

  /** Creates a new Shooter. */
  public Shooter() {
    propMotor = new CANSparkFlex(Constants.Shooter.propMotorCAN, MotorType.kBrushless);
    topMotor = new CANSparkFlex(Constants.Shooter.topMotorCAN, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(Constants.Shooter.bottomMotorCAN, MotorType.kBrushless);

    propMotor.setIdleMode(IdleMode.kCoast);
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    propEncoder = propMotor.getEncoder();
    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();
  }

  private void setPropSpeed(double speed){
    propSpeed = -speed;
  }
  private void setTopSpeed(double speed){
    topSpeed = speed;
  }
  private void setBottomSpeed(double speed){
    bottomSpeed = speed;
  }

  public void shoot(double speed){
    setPropSpeed(speed);
    setTopSpeed(speed);
    setBottomSpeed(speed);
  }
  public void ejectTop(double speed){
    setPropSpeed(speed);
    setTopSpeed(-speed);
    setBottomSpeed(speed);
  }
  public void ejectBottom(double speed){
    setPropSpeed(speed);
    setTopSpeed(speed);
    setBottomSpeed(-speed);
  }
  public void stopAll(){
    setPropSpeed(0);
    setTopSpeed(0);
    setBottomSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    propMotor.set(propSpeed);
    topMotor.set(topSpeed);
    bottomMotor.set(bottomSpeed);

    SmartDashboard.putNumber("Propellor Power", propMotor.get());
    SmartDashboard.putNumber("Propellor Velocity", propEncoder.getVelocity());
    SmartDashboard.putNumber("Top Power", topMotor.get());
    SmartDashboard.putNumber("Top Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Power", bottomMotor.get());
    SmartDashboard.putNumber("Bottom Velocity", bottomEncoder.getVelocity());
  }
}
