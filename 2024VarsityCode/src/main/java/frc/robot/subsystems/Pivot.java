// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* Desired Behavior:
 * I want this to mostly rely on automatic position control, but the specialist
 * can still have manual control.
 */

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.lib.math.Conversions;
import frc.lib.util.SparkUtilities;
import frc.robot.Constants;
import frc.robot.commands.SparkMaxPosition;

public class Pivot extends SubsystemBase {

  /* ATTRIBUTES */
  
  private CANSparkFlex rightPivot;
  private CANSparkFlex leftPivot;

  private AbsoluteEncoder pivotEncoder;

  private SparkPIDController pivotController;

  private TrapezoidProfile motionProfile;

  // Tested Positions
  public static final double INTAKE_POS = 70;
  public static final double HUMAN_POS = 60; // TODO
  public static final double AMP_POS = 90;


  /* CONSTRUCTORS */

  /** Creates a new Pivot. */
  public Pivot() {
    // Motor Setup
    rightPivot = new CANSparkFlex(Constants.Pivot.rightPivotCAN, MotorType.kBrushless);
    leftPivot = new CANSparkFlex(Constants.Pivot.leftPivotCAN, MotorType.kBrushless);

    rightPivot.follow(leftPivot);
    leftPivot.setInverted(true);

    leftPivot.setIdleMode(IdleMode.kBrake);
    rightPivot.setIdleMode(IdleMode.kBrake);

    SparkUtilities.optimizeFrames(rightPivot, false, false, true, false, false, false);
    SparkUtilities.optimizeFrames(leftPivot, true, false, true, false, false, true);

    // Encoder Setup
    pivotEncoder = leftPivot.getAbsoluteEncoder(Type.kDutyCycle);

    // PID Setup
    pivotController = leftPivot.getPIDController();
    pivotController.setFeedbackDevice(pivotEncoder);
    pivotController.setOutputRange(-1, 1);
    pivotController.setP(0.01);
    pivotController.setI(0.0);
    pivotController.setD(0.0);
    pivotController.setFF(0.0);

    motionProfile = new TrapezoidProfile(new Constraints(Conversions.dpsToRpm(30), Conversions.dpsToRpm(30)));
  }


  /* METHODS */
  /*
   * The way I want these to work is setSpeed will only be called by manual control, and
   * set position will only be called by auto control, and the single set positions (see
   * COMMANDS) should only be called by instant commands by button presses.
   */
  public void setPercOutput(double speed){
    leftPivot.set(speed);
  }
  public void setDesiredAngle(double position){
    pivotController.setReference(position, ControlType.kPosition);
  }

  public double getCurrentAngle(){
    return pivotEncoder.getPosition();
  }

  public void useState(TrapezoidProfile.State state){

  }
  public TrapezoidProfile.State getState(){
    return new TrapezoidProfile.State(pivotEncoder.getPosition(), pivotEncoder.getVelocity());
  }


  /* COMMANDS */

  // Set position commands
  public Command goTo(double position){
    return new SparkMaxPosition(leftPivot, position, 0, 2, this);
  }

  public Command profiledMove(double position){
    return new TrapezoidProfileCommand(
      motionProfile, 
      this::useState, 
      () -> new TrapezoidProfile.State(position, 0),
      this::getState,
      this).withName("Profiled Movement to "+position);
  }

  public Command setIntakePosition(){
    return new InstantCommand(() -> setDesiredAngle(INTAKE_POS), this);
  }
  public Command setDownPosition(){
    return new InstantCommand(() -> setDesiredAngle(2), this);
  }
  public Command setHumanIntakePosition(){
    return new InstantCommand(() -> setDesiredAngle(HUMAN_POS), this);
  }
  public Command setAmpPosition(){
    return new InstantCommand(() -> setDesiredAngle(AMP_POS), this);
  }

  public Command holdPositionThenZero(double position){
    return new FunctionalCommand(
      ()->setDesiredAngle(position), 
      ()->{}, 
      (interrupted)->setDesiredAngle(5.0), 
      ()->{return false;}, 
      this).withName("Holding Position - "+position);
  }
  public Command holdPosition(double position){
    return new FunctionalCommand(
      ()->setDesiredAngle(position), 
      ()->{}, 
      (interrupted)->{}, 
      ()->{return false;}, 
      this);
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Pivot Absolute Angle", pivotEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Speed", leftPivot.get());
    SmartDashboard.putString("Pivot Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());
  }
}
