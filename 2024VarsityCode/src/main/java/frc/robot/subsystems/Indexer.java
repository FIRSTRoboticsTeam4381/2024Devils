// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private CANSparkMax indexMotor;
  private double indexSpeed = 0.0;

  private DigitalInput indexEye;

  /** Creates a new Indexer. */
  public Indexer() {
    indexMotor = new CANSparkMax(Constants.Shooter.indexCAN, MotorType.kBrushless);

    indexEye = new DigitalInput(Constants.Shooter.indexDIO);
  }

  public void setSpeed(double speed){
    indexSpeed = speed;
  }
  public boolean getEye(){
    return !indexEye.get(); // TODO Break beam should return false when it is broken
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    indexMotor.set(indexSpeed);
  }
}
