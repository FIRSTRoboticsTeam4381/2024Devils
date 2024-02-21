// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkUtilities;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  /* ATTRIBUTES */

  private CANSparkMax intake;
  private CANSparkMax helper;

  public static final double INTAKE_SPEED = 0.5;


  /* CONSTRUCTORS */

  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(Constants.Intake.primaryIntakeCAN, MotorType.kBrushless);
    helper = new CANSparkMax(Constants.Intake.helperIntakeCAN, MotorType.kBrushless);

    helper.follow(intake);

    SparkUtilities.optimizeFrames(intake, true, false, false, false, false, false);
    SparkUtilities.optimizeFrames(helper, false, false, false, false, false, false);
  }


  /* METHODS */

  public void setPercOutput(double speed){
    intake.set(-speed);
  }


  /* COMMANDS */

  public InstantCommand start(){
    return new InstantCommand(() -> CommandScheduler.getInstance().schedule(run()));
  }
  public InstantCommand stop(){
    return new InstantCommand(() -> setPercOutput(0.0), this);
  }

  public Command run(){
    return new FunctionalCommand(
      ()->setPercOutput(INTAKE_SPEED),
      ()->{},
      (interrupt)->setPercOutput(0.0),
      ()->{return false;},
      this
    ).withName("Intaking");
  }
  /* Starts running the intake at a good speed. Doesn't need anything fancy, so this
  * just runs it on power output */
  public InstantCommand on(){
    return new InstantCommand(() -> setPercOutput(INTAKE_SPEED), this);
  }
  public InstantCommand off(){
    return new InstantCommand(() -> setPercOutput(0.0), this);
  }

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
  }
}
