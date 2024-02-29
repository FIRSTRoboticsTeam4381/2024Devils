// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.pathplanner.lib.controllers.PPRamseteController;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SparkUtilities.SparkUtilities;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  
  /* ATTRIBUTES */

  private CANSparkFlex propMotor;
  private CANSparkFlex topMotor;
  private CANSparkFlex bottomMotor;

  private RelativeEncoder propEncoder;
  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;

  private SparkPIDController propController;
  private SparkPIDController topController;
  private SparkPIDController bottomController;

  public static final double maxRPM = 6500;

  private double setpoint = 0.0;


  /* CONSTRUCTOR */
  /** Creates a new Shooter. */
  public Shooter() {
    // Motor Setup
    propMotor = new CANSparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);
    topMotor = new CANSparkFlex(Constants.Shooter.topCAN, MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(Constants.Shooter.bottomCAN, MotorType.kBrushless);

    propMotor.setSmartCurrentLimit(60);
    bottomMotor.setSmartCurrentLimit(30);
    topMotor.setSmartCurrentLimit(30);

    propMotor.setIdleMode(IdleMode.kCoast);
    topMotor.setIdleMode(IdleMode.kCoast);
    bottomMotor.setIdleMode(IdleMode.kCoast);

    propMotor.setInverted(true);
    topMotor.setInverted(true);
    bottomMotor.setInverted(false);
    //bottomMotor.follow(propMotor, true);

    SparkUtilities.optimizeFrames(propMotor, true, true, false, false, false, false);
    SparkUtilities.optimizeFrames(topMotor, false, true, false, false, false, false);
    SparkUtilities.optimizeFrames(bottomMotor, false, true, false, false, false, false);

    // Encoder Setup
    propEncoder = propMotor.getEncoder();
    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();

    resetPID();
  }


  /* ACCESSORS */

  /**
   * 
   * @return Current velocity of the propelling motor
   */
  public double getVelocity(){
    return propMotor.getEncoder().getVelocity();
  }

  /**
   * Decide if the shooter is ready for a note to be fed based on current velocity
   * @return Boolean determining if it is okay to feed a note to the shooter
   */
  public boolean readyForNote(){
    return (setpoint != 0 && (setpoint-Math.abs(propEncoder.getVelocity())<50 && setpoint-Math.abs(topEncoder.getVelocity())<50 && setpoint-Math.abs(bottomEncoder.getVelocity())<50));
  }

  public double getSetpoint(){
    return setpoint;
  }


  /* MUTATORS */

  /**
   * Set a flat percentage-based power output to the shooter motors. Doesn't run the PID.
   * @param speed Speed (-1.0 - 1.0) to output to the motors
   * @param deflect Whether the top motor should be flipped so as to allow the note to be deflected through the top
   */
  private void setPercOutput(double speed, boolean deflect){
    setpoint = speed*maxRPM;
    propMotor.set(speed);
    topMotor.set(speed * (deflect?-1:1));
    bottomMotor.set(speed);
  }

  /**
   * Set a reference velocity to be passed to the shooter motors. Runs off of a PID. Estimated time
   * to ramp to 1500 rpm = 2 seconds. Note: CAN Spark Flex theoretical max rpm = 6500
   * @param velocity
   * @param deflect Whether the top motor should be flipped so as to allow the note to be deflected through the top
   */
  public void setVelocity(double velocity, boolean deflect){
    setpoint = velocity;
    propController.setReference(velocity, ControlType.kVelocity);
    topController.setReference(velocity * (deflect?-1.0:1.0), ControlType.kVelocity);
    bottomController.setReference(velocity, ControlType.kVelocity);
  }

  public void setAmpVelocity(){
    setpoint = 600;
    propController.setReference(600, ControlType.kVelocity);
    topController.setReference(-1200, ControlType.kVelocity);
    bottomController.setReference(1200, ControlType.kVelocity);
  }
  


  /* INSTANT COMMANDS */

  /**
   * Pass a velocity reference to the shooter motors. For more information, see the setVelocity() function
   * of this class. Note: motors will continue running at this reference until a new command is called, so be careful
   * @param velocity Velocity to be set to the motors
   * @param deflect Whether the top motor should be flipped so as to allow the note to be deflected through the top
   * @return
   */
  public InstantCommand instantSetVelocityReference(double velocity, boolean deflect){
    return new InstantCommand(() -> setVelocity(velocity, deflect), this);
  }

  /**
   * Instant command to stop all shooter motors
   * @return
   */
  public InstantCommand instantStopAll(){
    setpoint = 0.0;
    return new InstantCommand(()->setPercOutput(0.0, false), this);
  }


  /* FUNCTIONAL COMMANDS */

  /**
   * Sets the velocity reference for the shooter to 1500 rpm, and maintains that velocity
   * until the command is interrupted. Has no natural end case, so must be interrupted
   * @return
   */
  public Command shootAvgSpeed(){
    return new FunctionalCommand(
      () -> setVelocity(1800.0, false), 
      () -> {}, 
      interrupted->setPercOutput(0.0, false), 
      ()->{return false;}, 
      this).withName("Holding Velocity "+1700);
  }

  /**
   * Spins the motors to deflect the note out through the top with a PID. Has no natural end case, so must
   * be interrupted. Motors will stop when command ends.
   * @return
   */
  public Command ampShoot(){
    return new FunctionalCommand(
      ()->setAmpVelocity(),
      ()->{},
      (interrupted)->setPercOutput(0, false),
      ()->{return false;},
      this
    ).withName("Amp Deflect");
  }

  /**
   * Spins shooter motors in reverse to assist in ejecting or to intake a note through the front. Uses a PID.
   * Has no natural end case, so must be interrupted. Motors will stop when command ends.
   * @return
   */
  public Command eject(){
    return new FunctionalCommand(
      ()->setVelocity(-1000.0, false), 
      ()->{}, 
      (interrupted)->setPercOutput(0.0, false), 
      ()->{return false;}, 
      this).withName("Ejecting");
  }


  public void resetPID(){
    // PID Setup
    double kp = 0.003;
    double ki = 0.0;
    double kd = 0.01;
    double kf = 0.00028;
    propController = propMotor.getPIDController();
    propController.setP(kp);
    propController.setI(ki);
    propController.setD(kd);
    propController.setFF(kf);
    propController.setOutputRange(-1, 1);

    kp = 0.003;
    ki = 0.0;
    kd = 0.01;
    kf = 0.00028;
    topController = topMotor.getPIDController();
    topController.setP(kp);
    topController.setI(ki);
    topController.setD(kd);
    topController.setFF(kf);
    topController.setOutputRange(-1, 1);

    bottomController = bottomMotor.getPIDController();
    bottomController.setP(kp);
    bottomController.setI(ki);
    bottomController.setD(kd);
    bottomController.setFF(kf);
    bottomController.setOutputRange(-1, 1);
  }


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("shooter/Propellor Velocity", propEncoder.getVelocity());
    SmartDashboard.putNumber("shooter/Top Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("shooter/Bottom Velocity", bottomEncoder.getVelocity());
    SmartDashboard.putString("shooter/Active Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());
    SmartDashboard.putNumber("shooter/Error", Math.abs(-setpoint-propEncoder.getVelocity()));
    SmartDashboard.putBoolean("shooter/Is Ready", readyForNote());
    SmartDashboard.putNumber("shooter/setpoint", setpoint);

    SmartDashboard.putNumber("shooter/Propellor Current", propMotor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/Top Current", topMotor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/Bottom Current", bottomMotor.getOutputCurrent());
  }


  public void burnFlash(){
    try{
      Thread.sleep(1000);
      propMotor.burnFlash();
      Thread.sleep(1000);
      topMotor.burnFlash();
      Thread.sleep(1000);
      bottomMotor.burnFlash();
      Thread.sleep(1000);
    }catch(InterruptedException e){
      DriverStation.reportError("Thread was interrupted while flashing shooter", e.getStackTrace());
    }
  }
}
