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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  private CANSparkFlex propMotor;
  private CANSparkFlex topMotor;
  private CANSparkFlex bottomMotor;

  private double propMotorSpeed = 0.0;
  private double topMotorSpeed = 0.0;
  private double bottomMotorSpeed = 0.0;

  public static final double maxRPM = 5000;

  /** Creates a new Shooter. */
  public Shooter() {
    propMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);
    topMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);

    propMotor.setIdleMode(IdleMode.kBrake);
    topMotor.setIdleMode(IdleMode.kBrake);
    bottomMotor.setIdleMode(IdleMode.kBrake);
  }

  private void setPropMotor(double speed){
    propMotorSpeed = speed;
  }
  private void setTopMotor(double speed){
    topMotorSpeed = speed;
  }
  private void setBottomMotor(double speed){
    bottomMotorSpeed = speed;
  }
  public void setAll(double speed, Mode mode){
    setPropMotor(speed);
    setTopMotor(mode==Mode.EJECT_TOP?-speed:speed);
    setBottomMotor(mode==Mode.EJECT_BOTTOM?-speed:speed);
  }
  public double getVelocity(){
    return propMotor.getEncoder().getVelocity();
  }
  public double getSetSpeed(){
    return propMotorSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    propMotor.set(propMotorSpeed);
    topMotor.set(topMotorSpeed);
    bottomMotor.set(bottomMotorSpeed);
  }

  public static enum Mode{SHOOT, EJECT_TOP, EJECT_BOTTOM}
}
