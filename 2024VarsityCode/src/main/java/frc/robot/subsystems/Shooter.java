// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkFlex propMotor;
  private CANSparkFlex topMotor;
  private CANSparkFlex bottomMotor;

  private double propSpeed = 0.0;
  private double topSpeed = 0.0;
  private double bottomSpeed = 0.0;

  private static final double SHOOT_SPEED = 0.5;
  private static final double EJECT_SPEED = 0.2;

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
    propMotor.set(-speed);
  }
  private void setTopSpeed(double speed){
    topMotor.set(-speed);
  }
  private void setBottomSpeed(double speed){
    bottomMotor.set(speed);
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

  public InstantCommand startShooter(){
    return new InstantCommand(() -> shoot(SHOOT_SPEED));
  }
  public InstantCommand startTopEject(){
    return new InstantCommand(() -> ejectTop(EJECT_SPEED));
  }
  public InstantCommand stopShooter(){
    return new InstantCommand(() -> shoot(0.0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Propellor Power", propMotor.get());
    SmartDashboard.putNumber("Propellor Velocity", propEncoder.getVelocity());
    SmartDashboard.putNumber("Top Power", topMotor.get());
    SmartDashboard.putNumber("Top Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Power", bottomMotor.get());
    SmartDashboard.putNumber("Bottom Velocity", bottomEncoder.getVelocity());
  }
}
