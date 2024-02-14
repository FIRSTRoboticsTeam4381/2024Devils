// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desired Behavior:
 * 
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.FlywheelRamp;

public class Shooter extends SubsystemBase {
  
  /* ATTRIBUTES */

  private CANSparkFlex propMotor;
  private CANSparkFlex topMotor;
  private CANSparkFlex bottomMotor;
  private SparkPIDController[] controllers; // TODO idk if this way of doing it will work

  public static final double maxRPM = 6500;

  private static final double EJECT_SPEED = 0.25; // TODO
  private static final double SHOOTING_SPEED = 0.5; // TODO


  /* CONSTRUCTOR */

  /** Creates a new Shooter. */
  public Shooter() {
    controllers = new SparkPIDController[3];
    propMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);
    topMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);

    controllers[0] = propMotor.getPIDController();
    controllers[1] = topMotor.getPIDController();
    controllers[2] = bottomMotor.getPIDController();

    propMotor.setIdleMode(IdleMode.kCoast);
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    for(int i = 0; i < controllers.length; i++){
      controllers[i].setP(0);
      controllers[i].setI(0);
      controllers[i].setD(0);
      controllers[i].setFF(0);
      controllers[i].setOutputRange(-1, 1);
    }
  }


  /* METHODS */
  public void setTopMotorSpeed(double speed){
    topMotor.set(speed); // TODO invert?
  }
  public void setBottomMotorSpeed(double speed){
    bottomMotor.set(speed); // TODO invert?
  }
  public void setPropMotorSpeed(double speed){
    propMotor.set(speed); // TODO invert?
  }
  public void shootAtSpeed(double speed){
    propMotor.set(speed);
    topMotor.set(speed);
    bottomMotor.set(speed);
  }
  public void ejectAtSpeed(double speed){
    propMotor.set(speed);
    topMotor.set(-speed);
    bottomMotor.set(speed);
  }
  public void setVelocity(double velocity){
    for(int i = 0; i < controllers.length; i++){
      controllers[i].setReference(velocity, ControlType.kVelocity);
    }
  }

  public double getVelocity(){
    return propMotor.getEncoder().getVelocity();
  }


  /* COMMANDS */

  // Instant commands
  public InstantCommand startEject(){
    return new InstantCommand(()->ejectAtSpeed(EJECT_SPEED), this);
  }
  public InstantCommand startShoot(){
    return new InstantCommand(()->shootAtSpeed(SHOOTING_SPEED), this);
  }
  public InstantCommand stopAll(){
    return new InstantCommand(()->shootAtSpeed(0.0), this);
  }

  // Functional commands
  public Command eject(){
    return new FunctionalCommand(
      ()->ejectAtSpeed(EJECT_SPEED),
      ()->{},
      (interrupted)->ejectAtSpeed(0.0),
      ()->{return false;},
      this
    ).withName("Ejecting");
  }

  public ConditionalCommand toggleShooter(){
    return new ConditionalCommand(shoot(), stopAll(), ()->{return propMotor.get()==0;});
  }
  public Command shoot(){
    return new FunctionalCommand(
      ()->shootAtSpeed(SHOOTING_SPEED),
      ()->{},
      (interrupted)->shootAtSpeed(0.0),
      ()->{return false;},
      this
    ).withName("Shooting");
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
