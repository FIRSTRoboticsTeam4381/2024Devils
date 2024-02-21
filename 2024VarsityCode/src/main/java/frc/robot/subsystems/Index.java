// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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

  private boolean noteStored(){
    return !indexEye.get();
  }

  private void setPercOutput(double speed){
    indexMotor.set(speed);
  }


  /* INSTANT COMMANDS */

  /**
   * Instant command to set 30% power to the index motor. Index will continue at this speed
   * until a new command is scheduled
   * @return
   */
  public InstantCommand instantStart(){
    return new InstantCommand(() -> setPercOutput(INDEX_SPEED), this);
  }

  /**
   * Instant command to set REVERSED 30% power to the index motor. Index will continue
   * at this speed until a new command is scheduled
   * @return
   */
  public InstantCommand instantStartEject(){
    return new InstantCommand(() -> setPercOutput(-INDEX_SPEED), this);
  }

  /**
   * Instant command to stop the index motor
   * @return
   */
  public InstantCommand instantStop(){
    return new InstantCommand(() -> setPercOutput(0.0), this);
  }


  /* FUNCTIONAL COMMANDS */

  /**
   * Turns index on and runs it at a constant speed until interrupted, at which point
   * it stops. Has no natural end condition, so must be interrupted
   * @return
   */
  public Command run(){
    return new FunctionalCommand(
      ()->setPercOutput(INDEX_SPEED), 
      ()->{}, 
      interrupted->setPercOutput(0.0), 
      ()->{return false;}, 
      this).withName("Running Index");
  }

  /**
   * Turns index on in reversed direction and runs at constant speed until interrupted,
   * at which point it stops. Has no natural end condition, so must be interrupted
   * @return
   */
  public Command eject(){
    return new FunctionalCommand(
      ()->setPercOutput(-INDEX_SPEED),
      ()->{},
      (interrupted)->setPercOutput(0.0),
      ()->{return false;},
      this
    ).withName("Ejecting");
  }

  /**
   * Turns index on and runs until a note is detected by the breakbeam or is interrupted.
   * @param reversed Which direction to intake through. If normally intaking, this is false,
   * if intaking through the shooter (backwards), this is true
   * @return
   */
  public Command indexUntilIn(boolean reversed){
    return new FunctionalCommand(
      ()->setPercOutput(INDEX_SPEED*(reversed?-1:1)),
      ()->{},
      (interrupted)->setPercOutput(0.0),
      this::noteStored,
      this
    ).withName("IndexUntilIn");
  }

  /**
   * Turns index on and runs until a note is NOT detected by the breakbeam or is interrupted.
   * @param reversed Which direction to intake through. If intaking normally, this is false.
   * @return
   */
  public Command indexUntilShot(boolean reversed){
    return new FunctionalCommand(
      ()->setPercOutput(INDEX_SPEED),
      ()->{},
      (interrupted)->setPercOutput(0.0),
      ()->{return !noteStored();},
      this
    ).withName("IndexUntilShot");
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("index/Note Stored", noteStored());
    SmartDashboard.putString("index/Active Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());
  }
}
