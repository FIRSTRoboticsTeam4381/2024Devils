// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {

  /* ATTRIBUTES */

  private SparkMax indexMotor;
  private DigitalInput[] eyes;

  private static final SparkMaxConfig MOTOR_CONFIG = new SparkMaxConfig();

  public static final double INDEX_SPEED = 0.75;


  /* CONSTRUCTORS */

  /** Creates a new Index. */
  public Index() {
    eyes = new DigitalInput[2];
    eyes[0] = new DigitalInput(Constants.Index.indexDIO1);
    eyes[1] = new DigitalInput(Constants.Index.indexDIO2);


    indexMotor = new SparkMax(Constants.Index.indexCAN, MotorType.kBrushless);

	MOTOR_CONFIG
		.smartCurrentLimit(30);

	indexMotor.configure(MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }


  /* METHODS */

  public boolean getEye(int index){
    return (index==0?eyes[index].get() : !eyes[index].get());
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
      ()->{return reversed?getEye(0):getEye(0)||getEye(1);},
      this
    ).withName("IndexUntilIn");
  }
  public Command indexUntilReady(boolean reversed){
    return new FunctionalCommand(
      ()->setPercOutput(INDEX_SPEED*0.75*(reversed?-1:1)),
      ()->{},
      (interrupted)->setPercOutput(0.0),
      ()->{return getEye(1);},
      this
    ).withName("IndexUntilReady");
  }

  /**
   * Turns index on and runs until a note is NOT detected by the breakbeam or is interrupted.
   * @param reversed Which direction to intake through. If intaking normally, this is false.
   * @return
   */
  public Command indexUntilShot(){
    return new FunctionalCommand(
      ()->setPercOutput(INDEX_SPEED*0.6),
      ()->{},
      (interrupted)->setPercOutput(0.0),
      ()->{return !getEye(0)&&!getEye(1);},
      this
    ).withName("IndexUntilShot");
  }

  public Command oneEyeIndex(boolean reversed){
    return new SequentialCommandGroup(
      indexUntilIn(reversed),
      new FunctionalCommand(
        ()->setPercOutput(INDEX_SPEED), 
        ()->{}, 
        (interrupted)->setPercOutput(0.0), 
        ()->{return !getEye(0);}, 
        this),
      indexUntilIn(true)
    );
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("index/Eye0", getEye(0));
    SmartDashboard.putBoolean("index/Eye1", getEye(1));
    SmartDashboard.putString("index/Active Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());

    SmartDashboard.putNumber("index/Index Current", indexMotor.getOutputCurrent());
  }
}
