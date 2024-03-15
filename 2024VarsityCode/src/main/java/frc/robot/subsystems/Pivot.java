// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkUtilities.SparkUtilities;
import frc.robot.Constants;
import frc.robot.commands.SparkPosition;

public class Pivot extends SubsystemBase {

  /* ATTRIBUTES */
  
  private CANSparkMax leader;
  private CANSparkMax follower;

  private AbsoluteEncoder shooterPivotEncoder;
  private AbsoluteEncoder motorPivotEncoder;

  private SparkPIDController pivotController;

  public class Positions{
    public static final double intake = 85;
    public static final double human = 115;
    public static final double amp = 90;
    public static final double transit = 10;
    public static final double podium = 34;
  }


  /* CONSTRUCTORS */

  /** Creates a new Pivot. */
  public Pivot() {
    // Motor Setup
    leader = new CANSparkMax(Constants.Pivot.rightPivotCAN, MotorType.kBrushless);
    follower = new CANSparkMax(Constants.Pivot.leftPivotCAN, MotorType.kBrushless);

    leader.setInverted(true);
    follower.follow(leader, true);
    
    leader.setIdleMode(IdleMode.kBrake);
    follower.setIdleMode(IdleMode.kBrake);

    leader.setSmartCurrentLimit(60);
    follower.setSmartCurrentLimit(60);

    SparkUtilities.optimizeFrames(leader, true, false, true, false, false, true);
    SparkUtilities.optimizeFrames(follower, false, false, true, false, false, true);

    // Encoder Setup
    /*
     * REV Software Level:
     * shooterPivotEncoder position conversion factor = 360
     * motorPivotEncoder position conversion factor = 360*48.0/50.0
     * make sure they move in the same direction, and figure out the offsets
     */
    shooterPivotEncoder = follower.getAbsoluteEncoder(Type.kDutyCycle);
    //shooterPivotEncoder.setPositionConversionFactor(360);
    motorPivotEncoder = leader.getAbsoluteEncoder(Type.kDutyCycle);
    //motorPivotEncoder.setPositionConversionFactor(360 * 48.0/50.0);

    // PID Setup
    pivotController = leader.getPIDController();
    pivotController.setFeedbackDevice(motorPivotEncoder);
    pivotController.setOutputRange(-1, 1);
    pivotController.setPositionPIDWrappingEnabled(true);
    pivotController.setPositionPIDWrappingMinInput(0);
    pivotController.setPositionPIDWrappingMaxInput(360);
    // Slot 0 = Regular Movement
    pivotController.setP(0.01, 0);
    pivotController.setI(0.0, 0);
    pivotController.setD(0.002, 0);
    pivotController.setFF(0.0005, 0);
    // Slot 1 = Auto Aiming - More aggressive and more precise
    pivotController.setP(0.017, 1);
    pivotController.setI(0.0, 1);
    pivotController.setD(0.008, 1);
    pivotController.setFF(0.0005, 1);
  }


  /* ACCESSORS */

  public double getAngle(){
    return getMotorAngle();
  }
  private double getShooterAngle(){
    return shooterPivotEncoder.getPosition();
  }
  private double getMotorAngle(){
    return motorPivotEncoder.getPosition();
  }


  /* MUTATORS */

  /**
   * Set a flat percentage-based output to the pivot. Useful for basic manual control
   * @param speed
   */
  public void manualControl(double speed){
    double pos = getAngle();
    if(pos < 15 || pos > 80) speed *= 0.6;
    leader.set(speed);
  }

  /**
   * Set a position reference for the pivot on a 0-360 degree range. Moves via PID control.
   * @param angle
   * @param slot the PID slot number to use. 0 is normal, 1 is auto aim (more aggressive)
   */
  public void setAngleReference(double angle, int slot){
    pivotController.setReference(angle, ControlType.kPosition, slot);
  }


  /* COMMANDS */

  /**
   * Using  tuned PID control, moves pivot to the passed angle. Releases control over pivot once it is within 3 degrees of the setpoint
   * @param angle Angle to travel to
   * @return
   */
  public Command goToAngle(double angle){
    return new SparkPosition(leader, angle, 1, 0.5, this, this::getAngle);
  }

  /**
   * Moves the pivot to a provided position until the command is interrupted, then releases control
   * @param position
   * @return
   */
  public Command holdPosition(double position){
    return new FunctionalCommand(
      ()->setAngleReference(position, 0), 
      ()->{}, 
      interrupted->{}, 
      ()->{return false;}, 
      this).withName("Holding position "+position);
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Position
    SmartDashboard.putNumber("pivot/absoluteAngle/shooter", getShooterAngle());
    SmartDashboard.putNumber("pivot/absoluteAngle/motor", getMotorAngle());
    // Command
    SmartDashboard.putString("pivot/Active Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());
    SmartDashboard.putBoolean("pivot/Aiming?", !(this.getCurrentCommand()==null||!this.getCurrentCommand().getName().equals("Auto Aim")));
    // Current Draw
    SmartDashboard.putNumber("pivot/Leader Current", leader.getOutputCurrent());
    SmartDashboard.putNumber("pivot/Follower Current", follower.getOutputCurrent());
  }
  
  public void burnFlash(){
    try{
      Thread.sleep(1000);
      leader.burnFlash();
      Thread.sleep(1000);
      follower.burnFlash();
      Thread.sleep(1000);
    }catch(InterruptedException e){
      DriverStation.reportError("Thread was interrupted while flashing pivot", e.getStackTrace());
    }
  }
}
