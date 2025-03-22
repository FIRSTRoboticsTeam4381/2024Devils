// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Shooter extends SubsystemBase {
  
  /* ATTRIBUTES */

  private SparkFlex propMotor;
  private SparkFlex topMotor;

  private static final SparkFlexConfig PROP_MOTOR_CONFIG = new SparkFlexConfig();
  private static final SparkFlexConfig TOP_MOTOR_CONFIG = new SparkFlexConfig();

  private RelativeEncoder propEncoder;
  private RelativeEncoder topEncoder;

  public static final double MAX_RPM = 6500;
  public static final double NORMAL_RPM = 2000; // A basic speed to use that works generally well for demos/etc

  private double setpoint = 0.0;
  //private boolean shootMode=true;
  private boolean trapMode=false;


  /* CONSTRUCTOR */
  /** Creates a new Shooter. */
  public Shooter() {
    // Motor Setup
    propMotor = new SparkFlex(Constants.Shooter.propCAN, MotorType.kBrushless);
    topMotor = new SparkFlex(Constants.Shooter.topCAN, MotorType.kBrushless);

    PROP_MOTOR_CONFIG
      .smartCurrentLimit(60)
      .idleMode(IdleMode.kCoast)
      .inverted(true);

	PROP_MOTOR_CONFIG.closedLoop
		.p(0.001, ClosedLoopSlot.kSlot0)
		.i(0.0, ClosedLoopSlot.kSlot0)
		.d(0.0, ClosedLoopSlot.kSlot0)
		.velocityFF(0.000165, ClosedLoopSlot.kSlot0)
		.outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0)
		.p(0.0005, ClosedLoopSlot.kSlot1)
		.i(0.0, ClosedLoopSlot.kSlot1)
		.d(0.0, ClosedLoopSlot.kSlot1)
		.velocityFF(0.00016, ClosedLoopSlot.kSlot1)
		.outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot1);

	propMotor.configure(PROP_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

	
	TOP_MOTOR_CONFIG
		.smartCurrentLimit(60)
		.idleMode(IdleMode.kCoast)
		.inverted(true);

	TOP_MOTOR_CONFIG.closedLoop
		.p(0.0005, ClosedLoopSlot.kSlot0)
		.i(0.0, ClosedLoopSlot.kSlot0)
		.d(0.0, ClosedLoopSlot.kSlot0)
		.velocityFF(0.00016, ClosedLoopSlot.kSlot0)
		.outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0)
		.p(0.00025, ClosedLoopSlot.kSlot1)
		.i(0.0, ClosedLoopSlot.kSlot1)
		.d(0.0, ClosedLoopSlot.kSlot1)
		.velocityFF(0.00016, ClosedLoopSlot.kSlot1)
		.outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot1);

	topMotor.configure(TOP_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode. kPersistParameters);

    // Encoder Setup
    propEncoder = propMotor.getEncoder();
    topEncoder = topMotor.getEncoder();

    propMotor.getEncoder().getVelocity();
    topMotor.getEncoder().getVelocity();
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
    return (setpoint != 0 && (setpoint-Math.abs(propEncoder.getVelocity())<(trapMode?100:300) && setpoint-Math.abs(topEncoder.getVelocity())<(trapMode?100:300)));
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
  public void setPercOutput(double speed, boolean deflect){
    setpoint = speed*MAX_RPM;
    propMotor.set(speed);
    topMotor.set(speed * (deflect?-1:1));
  }

  /**
   * Set a reference velocity to be passed to the shooter motors. Runs off of a PID. Estimated time
   * to ramp to 1500 rpm = 2 seconds. Note: CAN Spark Flex theoretical max rpm = 6500
   * @param velocity
   * @param deflect Whether the top motor should be flipped so as to allow the note to be deflected through the top
   */
  public void setVelocity(double velocity, boolean deflect){
    setpoint = velocity;
    propMotor.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    topMotor.getClosedLoopController().setReference(velocity * (deflect?-1.0:1.0), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    trapMode=false;
  }

  public void setAmpVelocity(){
    setpoint = 1800;
    propMotor.getClosedLoopController().setReference(1800, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    propMotor.getClosedLoopController().setReference(-4500, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    trapMode=false;
  }
  public void setTrapVelocity(){
    setpoint = 1400;
    propMotor.getClosedLoopController().setReference(1400, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    propMotor.getClosedLoopController().setReference(-4000, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
    trapMode=true;
  }

  public void setCurrentLimit(int current1, int current2){
	PROP_MOTOR_CONFIG.smartCurrentLimit(current1);
	TOP_MOTOR_CONFIG.smartCurrentLimit(current2);

	propMotor.configure(PROP_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	topMotor.configure(TOP_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
  public Command shoot(double rpm){
    return new FunctionalCommand(
      () -> setVelocity(rpm, false), 
      () -> {}, 
      interrupted->setPercOutput(0.0, false), 
      ()->{return false;}, 
      this).withName("Holding Velocity "+rpm);
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
  public Command trapShoot(){
    return new FunctionalCommand(
      ()->{setCurrentLimit(80,80);setTrapVelocity();},
      ()->{},
      (interrupted)->{setCurrentLimit(60, 60);setPercOutput(0, false);},
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
      ()->setVelocity(-1800.0, false), 
      ()->{}, 
      (interrupted)->setPercOutput(0.0, false), 
      ()->{return false;}, 
      this).withName("Ejecting");
  }

  public Command ejectFromAmp(){
    return new FunctionalCommand(
      ()->{setCurrentLimit(80,80);setVelocity(-2000.0, true);}, 
      ()->{}, 
      (interrupted)->{setCurrentLimit(60,60);setPercOutput(0.0, false);}, 
      ()->{return false;}, 
      this).withName("Eject from amp");
  }

  /*
  public Command customBangBang(){
    return new SequentialCommandGroup(
      new FunctionalCommand(
        () -> setPercOutput(1.0, false), 
        ()->{}, 
        (interrupted)->{if(interrupted){setPercOutput(0.0, false);}}, 
        ()->{return 1800-propEncoder.getVelocity()<50&&1800-topEncoder.getVelocity()<50&&bottomEncoder.getVelocity()<50;}, 
        this),
      shootAvgSpeed()
    );
  }
  */


  /* PERIODIC */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("shooter/Propellor Velocity", propEncoder.getVelocity());
    SmartDashboard.putNumber("shooter/Top Velocity", topEncoder.getVelocity());
    SmartDashboard.putString("shooter/Active Command", this.getCurrentCommand()==null?"None":this.getCurrentCommand().getName());
    SmartDashboard.putNumber("shooter/Error", Math.abs(-setpoint-propEncoder.getVelocity()));
    SmartDashboard.putBoolean("shooter/Shooter Ready", readyForNote());
    SmartDashboard.putBoolean("shooter/Shooter Running", setpoint!=0.0);
    SmartDashboard.putNumber("shooter/setpoint", setpoint);

    SmartDashboard.putNumber("shooter/Propellor Current", propMotor.getOutputCurrent());
    SmartDashboard.putNumber("shooter/Top Current", topMotor.getOutputCurrent());
  }
}
