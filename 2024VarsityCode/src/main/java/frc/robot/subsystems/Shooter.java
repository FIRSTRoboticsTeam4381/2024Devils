// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desired Behavior:
 * 
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.FlywheelRamp;

public class Shooter extends SubsystemBase {
  private CANSparkFlex propMotor;
  private CANSparkFlex topMotor;
  private CANSparkFlex bottomMotor;

  public static final double maxRPM = 6500;

  private static final double EJECT_SPEED = 0.5; // TODO
  private static final double SHOOTING_SPEED = 0.5; // TODO

  private boolean shooting = false;

  /** Creates a new Shooter. */
  public Shooter() {
    propMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);
    topMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);

    propMotor.setIdleMode(IdleMode.kCoast);
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setPropSpeed(double speed){
    propMotor.set(speed);
  }
  public void setTopSpeed(double speed){
    topMotor.set(speed);
  }
  public void setBottomSpeed(double speed){
    bottomMotor.set(speed);
  }

  public void setAll(double speed, boolean shooting){
    setPropSpeed(speed);
    setBottomSpeed(speed);
    setTopSpeed(shooting?speed:-speed);
  }

  public double getVelocity(){
    return propMotor.getEncoder().getVelocity();
  }

  public boolean getShooting(){
    return shooting;
  }

  public ConditionalCommand toggleShooter(){
    return new ConditionalCommand(startShooter(), stop(), this::getShooting);
  }
  
  public SequentialCommandGroup startShooter(){
    shooting = true;
    return new SequentialCommandGroup(
      new FlywheelRamp(this, SHOOTING_SPEED*maxRPM),
      new InstantCommand(() -> setAll(SHOOTING_SPEED, true), this)
    );
  }

  public InstantCommand startEjecting(){
    return new InstantCommand(() -> setAll(EJECT_SPEED, false), this);
  }

  public InstantCommand stop(){
    shooting = false;
    return new InstantCommand(() -> setAll(0, true), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
