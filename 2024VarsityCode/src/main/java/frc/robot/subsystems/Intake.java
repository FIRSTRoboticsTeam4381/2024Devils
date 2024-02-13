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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  /* ATTRIBUTES */

  private CANSparkMax primaryIntake;
  private CANSparkMax helperIntake;
  public static final double INTAKE_SPEED = 0.5;


  /* CONSTRUCTORS */

  /** Creates a new Intake. */
  public Intake() {
    primaryIntake = new CANSparkMax(Constants.Intake.primaryIntakeCAN, MotorType.kBrushless);
    helperIntake = new CANSparkMax(Constants.Intake.helperIntakeCAN, MotorType.kBrushless);
  }


  /* METHODS */

  public void setIntakeSpeed(double speed){
    primaryIntake.set(speed);
    helperIntake.set(speed);
  }


  /* COMMANDS */

  public Command run(){
    return new FunctionalCommand(
      ()->setIntakeSpeed(INTAKE_SPEED),
      ()->{},
      (interrupt)->setIntakeSpeed(0.0),
      ()->{return false;},
      this
    ).withName("intakeRun");
  }
  /* Starts running the intake at a good speed. Doesn't need anything fancy, so this
  * just runs it on power output */
  public InstantCommand start(){
    return new InstantCommand(() -> setIntakeSpeed(INTAKE_SPEED), this);
  }
  public InstantCommand stop(){
    return new InstantCommand(() -> setIntakeSpeed(0.0), this);
  }


  /* PERIODIC */
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
