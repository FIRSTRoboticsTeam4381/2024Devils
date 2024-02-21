// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desired Behavior:
 * 
 */

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Limelight;
import frc.lib.util.SparkUtilities;
import frc.robot.Constants;
import frc.robot.commands.FlywheelRamp;

public class Shooter extends SubsystemBase {
  
  /* ATTRIBUTES */

  private CANSparkFlex propMotor;
  private CANSparkFlex topMotor;
  private CANSparkFlex bottomMotor;
  private SparkPIDController[] controllers;

  private RelativeEncoder propEncoder;
  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;

  public static final double maxRPM = 6500;

  private static final double EJECT_SPEED = 0.2;
  private static final double SHOOTING_SPEED = 0.5;

  private boolean ampMode = false;
  private double setpoint = 0.0;


  /* CONSTRUCTOR */

  /** Creates a new Shooter. */
  public Shooter() {
    // Motor Setup
    propMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);
    topMotor = new CANSparkFlex(Constants.Shooter.topCAN, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(Constants.Shooter.bottomCAN, MotorType.kBrushless);

    propMotor.setIdleMode(IdleMode.kCoast);
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    SparkUtilities.optimizeFrames(propMotor, false, true, false, false, false, false);
    SparkUtilities.optimizeFrames(topMotor, false, true, false, false, false, false);
    SparkUtilities.optimizeFrames(bottomMotor, false, true, false, false, false, false);

    // Encoder Setup
    propEncoder = propMotor.getEncoder();
    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();

    // PID Setup
    controllers = new SparkPIDController[] {propMotor.getPIDController(), topMotor.getPIDController(), bottomMotor.getPIDController()};

    // TODO configure
    for(int i = 0; i < controllers.length; i++){
      controllers[i].setP(0.001);
      controllers[i].setI(0.0);
      controllers[i].setD(0.0);
      controllers[i].setFF(0.000212);
      controllers[i].setOutputRange(-1, 1);
    }
  }


  /* METHODS */
  public void setTopMotorPercOutput(double speed){
    topMotor.set(-speed); // TODO invert?
  }
  public void setBottomMotorPercOutput(double speed){
    bottomMotor.set(speed); // TODO invert?
  }
  public void setPropMotorPercOutput(double speed){
    propMotor.set(speed); // TODO invert?
  }
  public void setPercOutput(double speed){
    setPropMotorPercOutput(speed);
    setTopMotorPercOutput(speed);
    setBottomMotorPercOutput(speed);
  }
  public void ejectPercOutput(double speed){
    setPropMotorPercOutput(speed);
    setTopMotorPercOutput(-speed);
    setBottomMotorPercOutput(speed);
  }

  public void setVelocity(double velocity){
    setpoint = velocity;
    propMotor.getPIDController().setReference(-velocity, ControlType.kVelocity);
    topMotor.getPIDController().setReference(-velocity, ControlType.kVelocity);
    bottomMotor.getPIDController().setReference(velocity, ControlType.kVelocity);
  }
  public double getVelocity(){
    return propMotor.getEncoder().getVelocity();
  }

  public boolean readyForNote(){
    return (ampMode || Math.abs(setpoint-propEncoder.getVelocity())<50);
  }

  // TODO TEST
  public double getDistanceFromGoal(){
    boolean isBlue = DriverStation.getAlliance().get() == Alliance.Blue;

    double[] position = isBlue ? Limelight.getBlueRelativePosition() : Limelight.getRedRelativePosition();
    double[] goalPosition = {isBlue?0:0,isBlue?0:0}; // TODO get goal positions relative to origins (idk what the origins are);
    double distance = Math.sqrt(Math.pow(position[0]-goalPosition[0],2)+Math.pow(position[1]-goalPosition[1],2));
    return distance;
  }


  /* COMMANDS */

  // Instant commands
  public InstantCommand setVelocityReference(double velocity){
    ampMode = false;
    return new InstantCommand(() -> setVelocity(velocity), this);
  }
  public InstantCommand stopAll(){
    return new InstantCommand(()->setPercOutput(0.0), this);
  }

  public Command eject(){
    return new FunctionalCommand(
      ()->setPercOutput(-0.25), 
      ()->{}, 
      (interrupted)->setPercOutput(0.0), 
      ()->{return false;}, 
      this).withName("Ejecting");
  }

  // Functional commands
  public Command ampShoot(){
    ampMode = true;
    return new FunctionalCommand(
      ()->ejectPercOutput(EJECT_SPEED),
      ()->{},
      (interrupted)->ejectPercOutput(0.0),
      ()->{return false;},
      this
    ).withName("Amp");
  }

  public Command shootAvg(){
    ampMode = false;
    return new FunctionalCommand(
      () -> setVelocity(1500), 
      () -> {}, 
      interrupted->setVelocity(0), 
      ()->{return false;}, 
      this).withName("Holding Velocity"+1500);
  }

  public Command shootWithRamp(){
    ampMode = false;
    return new SequentialCommandGroup(
      new FlywheelRamp(this, 1400, 0).withName("Shooter Ramp"), // Stops when cancelled, doesn't otherwise
      new FunctionalCommand(
        () -> setVelocity(1500), 
        () -> {}, 
        interrupted->setVelocity(0), 
        ()->{return false;}, 
        this).withName("Holding Velocity") // Stops when cancelled
    );
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Propellor Power", propMotor.get());
    SmartDashboard.putNumber("Propellor Velocity", propEncoder.getVelocity());
    SmartDashboard.putNumber("Top Power", topMotor.get());
    SmartDashboard.putNumber("Top Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Power", bottomMotor.get());
    SmartDashboard.putNumber("Bottom Velocity", bottomEncoder.getVelocity());
    SmartDashboard.putString("Shooter Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());
  }
}
