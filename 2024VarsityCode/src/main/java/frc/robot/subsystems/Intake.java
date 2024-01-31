// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desired Behavior:
 * Intake should be a toggle triggered by a command. Will happen in sequence with other actions,
 * so intake speed will changed with instant commands in command groups. Otherwise, intake should
 * run at its set speed
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax primaryIntake;
  private CANSparkMax helperIntake;

  private double primaryIntakeSpeed = 0.0;
  private double helperIntakeSpeed = 0.0;

  /** Creates a new Intake. */
  public Intake() {
    primaryIntake = new CANSparkMax(Constants.Intake.primaryIntakeCAN, MotorType.kBrushless);
    helperIntake = new CANSparkMax(Constants.Intake.helperIntakeCAN, MotorType.kBrushless);
  }

  public void setIntakeSpeed(double speed){
    primaryIntakeSpeed = speed;
    helperIntakeSpeed = speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    primaryIntake.set(primaryIntakeSpeed);
    helperIntake.set(helperIntakeSpeed);
  }
}
