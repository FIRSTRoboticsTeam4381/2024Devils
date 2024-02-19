// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desired Behavior:
 * When the toggle button is pressed to turn the intake on, it also starts the index motor.
 * The motor will run until the break beam is broken, and this will then stop both the index
 * and intake motors. Will be triggered with the same groups as intake. Index should just
 * always run at its set speed, and commands just change the set speed
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkUtilities;
import frc.robot.Constants;

public class Index extends SubsystemBase {

  /* ATTRIBUTES */

  private CANSparkMax indexMotor;
  private DigitalInput indexEye;

  public static final double INDEX_SPEED = 0.3;


  /* CONSTRUCTORS */

  /** Creates a new Index. */
  public Index() {
    indexMotor = new CANSparkMax(Constants.Index.indexCAN, MotorType.kBrushless);
    indexEye = new DigitalInput(Constants.Index.indexDIO);

    SparkUtilities.optimizeFrames(indexMotor, false, false, false, false, false, false);
  }


  /* METHODS */

  public boolean noteStored(){
    return !indexEye.get();
  }

  public void setPercOutput(double speed){
    indexMotor.set(speed);
  }


  /* COMMANDS */

  // Instant Commands
  public InstantCommand start(){
    return new InstantCommand(() -> setPercOutput(INDEX_SPEED), this);
  }
  public InstantCommand stop(){
    return new InstantCommand(() -> setPercOutput(0.0), this);
  }
  public InstantCommand startEject(){
    return new InstantCommand(() -> setPercOutput(-INDEX_SPEED), this);
  }

  // Full Commands
  public Command indexUntilIn(){
    return new FunctionalCommand(
      ()->setPercOutput(INDEX_SPEED),
      ()->{},
      (interrupted)->setPercOutput(0.0),
      this::noteStored,
      this
    ).withName("IndexUntilIn");
  }

  public Command indexUntilShot(){
    return new FunctionalCommand(
      ()->setPercOutput(INDEX_SPEED),
      ()->{},
      (interrupted)->setPercOutput(0.0),
      ()->{return !noteStored();},
      this
    ).withName("IndexUntilShot");
  }

  public Command eject(){
    return new FunctionalCommand(
      ()->setPercOutput(-INDEX_SPEED),
      ()->{},
      (interrupted)->setPercOutput(0.0),
      ()->{return false;},
      this
    ).withName("Ejecting");
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Index Power", indexMotor.get());
    SmartDashboard.putBoolean("Note Inside", noteStored());
  }
}
