// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private CANSparkMax indexMotor;
  private double indexSpeed = 0.0;
  private Intake intake;

  private RelativeEncoder indexEncoder;
  private DigitalInput indexEye;

  public static final double INDEX_SPEED = 1.0;

  /** Creates a new Indexer. */
  public Indexer(Intake intake) {
    indexMotor = new CANSparkMax(Constants.Index.indexCAN, MotorType.kBrushless);
    this.intake = intake;

    indexEncoder = indexMotor.getEncoder();
    indexEye = new DigitalInput(Constants.Index.indexDIO);
  }

  public void setSpeed(double speed){
    indexSpeed = speed;
  }

  public boolean noteStored(){
    return !indexEye.get();
  }

  public InstantCommand startIntake(){
    return new InstantCommand(() -> {intake.setIntakeSpeed(Intake.INTAKE_SPEED); setSpeed(Indexer.INDEX_SPEED);});
  }
  public InstantCommand startEject(){
    return new InstantCommand(() -> {intake.setIntakeSpeed(-Intake.INTAKE_SPEED); setSpeed(-Indexer.INDEX_SPEED);});
  }
  public InstantCommand stopIntake(){
    return new InstantCommand(() -> {intake.setIntakeSpeed(0.0); setSpeed(0.0);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    indexMotor.set(indexSpeed);

    SmartDashboard.putNumber("Index Power", indexMotor.get());
    SmartDashboard.putBoolean("Note Inside", noteStored());
  }
}
