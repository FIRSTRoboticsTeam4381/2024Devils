// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desired Behavior:
 * Eventually, I want this to be able to be moved via both manual control
 * and position control. I'm not entirely sure how to achieve that yet
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private CANSparkMax baseMotor;
  private CANSparkMax elbowMotor;

  

  /** Creates a new Climb. */
  public Climb() {
    baseMotor = new CANSparkMax(Constants.Climb.baseMotorCAN, MotorType.kBrushless);
    elbowMotor = new CANSparkMax(Constants.Climb.elbowMotorCAN, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
