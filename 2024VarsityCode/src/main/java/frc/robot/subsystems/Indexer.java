// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkOptimizer;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
  private CANSparkMax indexMotor;
  private Intake intake;
  private Shooter shooter;

  private RelativeEncoder indexEncoder;
  private DigitalInput indexEye;

  public static final double INDEX_SPEED = 0.5;

  /** Creates a new Indexer. */
  public Indexer(Intake intake, Shooter shooter) {
    indexMotor = new CANSparkMax(Constants.Index.indexCAN, MotorType.kBrushless);
    this.intake = intake;
    this.shooter = shooter;

    indexEncoder = indexMotor.getEncoder();
    indexEye = new DigitalInput(Constants.Index.indexDIO);

    SparkOptimizer.optimizeFrames(indexMotor, false, false, false, false, false, false);
  }

  public void setSpeed(double speed){
    indexMotor.set(speed);
  }

  public void feedIfReady(double speed){
    if(shooter.isRunning()&&indexMotor.get()==0.0){
      setSpeed(speed);
    }
  }

  public boolean noteStored(){
    return !indexEye.get();
  }

  public InstantCommand startIntake(){
    return new InstantCommand(() -> {intake.setIntakeSpeed(Intake.INTAKE_SPEED); setSpeed(INDEX_SPEED);});
  }
  public InstantCommand startEject(){
    return new InstantCommand(() -> {intake.setIntakeSpeed(-Intake.INTAKE_SPEED); setSpeed(-INDEX_SPEED);});
  }
  public InstantCommand stopIntake(){
    return new InstantCommand(() -> {intake.setIntakeSpeed(0.0); setSpeed(0.0);});
  }

  public Command intake(){
    return new FunctionalCommand(
      () -> {intake.setIntakeSpeed(Intake.INTAKE_SPEED); setSpeed(INDEX_SPEED);},
      () -> {},
      (interrupted) -> {intake.setIntakeSpeed(0.0); setSpeed(0.0);}, 
      this::noteStored,
      intake, this);
  }

  public Command feedNote(){
    return new FunctionalCommand(
      () -> {}, 
      () -> feedIfReady(INDEX_SPEED), 
      (interrupted) -> setSpeed(0.0), 
      () -> !noteStored(), 
      this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Index Power", indexMotor.get());
    SmartDashboard.putBoolean("Note Inside", noteStored());
  }
}
