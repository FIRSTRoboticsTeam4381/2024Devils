// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkUtilities.SparkUtilities;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  /* ATTRIBUTES */

  private CANSparkMax intake;
  private CANSparkMax helper;

  public static final double INTAKE_SPEED = 1.0;


  /* CONSTRUCTORS */

  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(Constants.Intake.primaryIntakeCAN, MotorType.kBrushless);
    helper = new CANSparkMax(Constants.Intake.helperIntakeCAN, MotorType.kBrushless);
    intake.setSmartCurrentLimit(40);
    helper.setSmartCurrentLimit(30);

    helper.setInverted(false);

    //helper.follow(intake);

    SparkUtilities.optimizeFrames(intake, false, false, false, false, false, false);
    SparkUtilities.optimizeFrames(helper, false, false, false, false, false, false);
  }


  /* METHODS */

  private void setPercOutput(double speed){
    intake.set(-speed);
    helper.set(-speed*0.4);
  }



  /* INSTANT COMMANDS */
  // Shouldn't use these, more just for testing. Use the functional commands

  /**
   * Turns intake on. Is an instant command, so intake will continue running until a new command is scheduled
   * @return
   */
  public InstantCommand instantStart(){
    return new InstantCommand(() -> setPercOutput(INTAKE_SPEED), this);
  }

  /**
   * Turns intake off. This is an instant command, useful for instantly cancelling other intake commands
   * @return
   */
  public InstantCommand instantStop(){
    return new InstantCommand(() -> setPercOutput(0.0), this);
  }


  /* FUNCTIONAL COMMANDS */

  /**
   * Turns intake on and runs it at a constant speed until the command is interrupted. This has no
   * natural end condition, so it must be interrupted to stop the intake.
   * @return
   */
  public Command run(){
    return new FunctionalCommand(
      ()->setPercOutput(INTAKE_SPEED),
      ()->{},
      (interrupt)->setPercOutput(0.0),
      ()->{return false;},
      this
    ).withName("Intaking");
  }

  /**
   * Turns intake on at the same speed as intaking but with the direction reversed. Has no
   * natural end condition, so it must be interrupted to stop the intake
   * @return
   */
  public Command eject(){
    return new FunctionalCommand(
      ()->setPercOutput(-INTAKE_SPEED),
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
    SmartDashboard.putString("intake/Active Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());

    SmartDashboard.putNumber("intake/Intake Current", intake.getOutputCurrent());
    SmartDashboard.putNumber("intake/Helper Current", helper.getAppliedOutput());
  }


  public void burnFlash(){
    try{
      Thread.sleep(1000);
      intake.burnFlash();
      Thread.sleep(1000);
      helper.burnFlash();
      Thread.sleep(1000);
    }catch(InterruptedException e){
      DriverStation.reportError("Thread was interrupted while flashing intake", e.getStackTrace());
    }
  }
  
}
