// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;

import frc.lib.math.Conversions;
import frc.lib.util.SparkUtilities.SparkUtilities;
import frc.robot.Constants;
import frc.robot.commands.SparkPosition;

public class Pivot extends SubsystemBase {

  /* ATTRIBUTES */
  
  private CANSparkMax rightPivot;
  private CANSparkMax leftPivot;

  private AbsoluteEncoder pivotEncoder;

  private SparkPIDController pivotController;

  private TrapezoidProfile motionProfile;

  // Tested Positions
  public static final double INTAKE_POS = 85;
  public static final double HUMAN_POS = 115;
  public static final double AMP_POS = 92;
  public static final double TRANSIT_POS = 10;

  private double MAX = 110.0; // TODO

  private double posReference = 0.0;


  /* CONSTRUCTORS */

  /** Creates a new Pivot. */
  public Pivot() {
    // Motor Setup
    leftPivot = new CANSparkMax(Constants.Pivot.leftPivotCAN, MotorType.kBrushless);
    rightPivot = new CANSparkMax(Constants.Pivot.rightPivotCAN, MotorType.kBrushless);
    leftPivot.setInverted(false);
    rightPivot.follow(leftPivot, true);
    leftPivot.setIdleMode(IdleMode.kBrake);
    rightPivot.setIdleMode(IdleMode.kBrake);
    leftPivot.setSmartCurrentLimit(40);
    rightPivot.setSmartCurrentLimit(40);

    SparkUtilities.optimizeFrames(leftPivot, true, false, true, false, false, true);
    SparkUtilities.optimizeFrames(rightPivot, false, false, true, false, false, false);

    // Encoder Setup
    pivotEncoder = leftPivot.getAbsoluteEncoder(Type.kDutyCycle);

    // PID Setup
    pivotController = leftPivot.getPIDController();
    pivotController.setFeedbackDevice(pivotEncoder);
    pivotController.setOutputRange(-1, 1);
    pivotController.setPositionPIDWrappingEnabled(true);
    pivotController.setPositionPIDWrappingMinInput(0);
    pivotController.setPositionPIDWrappingMaxInput(360);
    pivotController.setP(0.008, 0);
    pivotController.setI(0.0, 0);
    pivotController.setD(0.002, 0);
    pivotController.setFF(0.0001, 0);

    pivotController.setP(0.0125, 1);
    pivotController.setI(0.000001, 1);
    pivotController.setD(0.01, 1);
    pivotController.setFF(0.0004, 1);

    // Trapezoid Profile Setup
    motionProfile = new TrapezoidProfile(new Constraints(Conversions.dpsToRpm(4000 * (125.0*50.0/48.0)), Conversions.dpsToRpm(1500 * (125.0*50.0/48.0))));
    // Torque = 1:5 + 1:5 + 1:5 + 48:50 = 130.21 : 1
    // RPM = 5:1 + 5:1 + 5:1 + 50:48 = 1 : 130.20833333

  }


  /* ACCESSORS */

  /**
   * Get the current angle of the pivot encoder
   * @return
   */
  public double getAngle(){
    return pivotEncoder.getPosition();
  }

  /**
   * Get the current Trapezoid Profile state of the pivot arm. Useful for profiled motion
   * @return
   */
  public TrapezoidProfile.State getState(){
    return new TrapezoidProfile.State(pivotEncoder.getPosition(), pivotEncoder.getVelocity());
  }


  /* MUTATORS */

  /**
   * Set a flat percentage-based output to the pivot. Useful for basic manual control
   * @param speed
   */
  public void setPercOutput(double speed){
    double pos = pivotEncoder.getPosition();
    if(pos < 10 || pos > 80) speed *= 0.75;
    leftPivot.set(speed);
  }

  /**
   * Set a position reference for the pivot on a 0-360 degree range. Moves via PID control.
   * @param angle
   * @param slot the PID slot number to use. 0 is normal, 1 is auto aim (more aggressive)
   */
  public void setAngleReference(double angle, int slot){
    pivotController.setReference(angle, ControlType.kPosition, slot);
    posReference = angle;
  }

  /**
   * Method to accept a Trapezoid Profile state and set the PID position reference to that state.
   * @param state
   */
  public void useState(TrapezoidProfile.State state){
    setAngleReference(state.position, 0);
  }


  /* COMMANDS */
  // No instant commands here, because it will immediately hand off to manual pivot, cancelling the set point making it useless

  /**
   * Using  tuned PID control, moves pivot to the passed angle. Releases control over pivot once it is within 3 degrees of the setpoint
   * @param angle Angle to travel to
   * @return
   */
  private Command goToAngle(double angle){
    return new SparkPosition(leftPivot, angle, 0, 3, this, pivotEncoder::getPosition);
  }
  public Command goToIntake(){
    return goToAngle(INTAKE_POS).withName("Moving to intake");
  }
  public Command goToHuman(){
    return goToAngle(HUMAN_POS).withName("Moving to human");
  }
  public Command goToTransit(){
    return goToAngle(TRANSIT_POS).withName("Moving to transit");
  }
  public Command goToAmp(){
    return goToAngle(AMP_POS).withName("Moving to amp");
  }

  /**
   * Moves the pivot to a provided position until the command is interrupted, at which point the pivot will return back to the transit position
   * @param position
   * @return
   */
  public Command goToTemporaryPosition(double position){
    return new FunctionalCommand(
      ()->setAngleReference(position, 0), 
      ()->{}, 
      interrupted->setAngleReference(TRANSIT_POS, 0), 
      ()->{return false;}, 
      this).withName("Holding position "+position);
  }

  /**
   * Move the pivot to a position via a Trapezoid Motion profile. This profile has yet to be tested and configured
   * @param position
   * @return
   */
  public Command profiledMove(double position){
    return new TrapezoidProfileCommand(
      motionProfile, 
      this::useState, 
      () -> new TrapezoidProfile.State(position, 0.0),
      this::getState,
      this).withName("Profiled Movement to "+position);
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("pivot/Absolute Angle", pivotEncoder.getPosition());
    SmartDashboard.putNumber("pivot/Pivot Velocity", pivotEncoder.getVelocity());
    SmartDashboard.putNumber("pivot/Relative Position", leftPivot.getEncoder().getPosition());
    SmartDashboard.putString("pivot/Active Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());

    SmartDashboard.putNumber("pivot/Left Current", leftPivot.getOutputCurrent());
    SmartDashboard.putNumber("pivot/Right Current", rightPivot.getOutputCurrent());
    SmartDashboard.putNumber("pivot/Position Reference", posReference);

    //if(leftPivot.getEncoder().getPosition() >= MAX)
  }


  public void burnFlash(){
    try{
      Thread.sleep(1000);
      leftPivot.burnFlash();
      Thread.sleep(1000);
      rightPivot.burnFlash();
      Thread.sleep(1000);
    }catch(InterruptedException e){
      DriverStation.reportError("Thread was interrupted while flashing pivot", e.getStackTrace());
    }
  }
}
