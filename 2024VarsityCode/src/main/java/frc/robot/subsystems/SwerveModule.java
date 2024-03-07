package frc.robot.subsystems; 
 
import com.revrobotics.CANSparkFlex; 
import com.revrobotics.CANSparkMax; 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.SparkAbsoluteEncoder; 
import com.revrobotics.SparkPIDController; 
import com.revrobotics.CANSparkBase.ControlType; 
import com.revrobotics.CANSparkLowLevel.MotorType; 
 
import frc.lib.math.Conversions; 
import frc.lib.util.LogOrDash;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.SparkUtilities.SparkUtilities;
import frc.robot.Constants; 

import edu.wpi.first.math.controller.SimpleMotorFeedforward; 
import edu.wpi.first.math.geometry.Rotation2d; 
import edu.wpi.first.math.kinematics.SwerveModulePosition; 
import edu.wpi.first.math.kinematics.SwerveModuleState; 
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
 
public class SwerveModule { 
 
    /* ATTRIBUTES */ 
    public int moduleNumber; 
    private CANSparkMax mAngleMotor; 
    private CANSparkFlex mDriveMotor; 
 
    private RelativeEncoder mDriveEncoder; 
    private SparkAbsoluteEncoder mAngleEncoder;

    private SparkPIDController drivePIDController;
    private SparkPIDController anglePIDController;
 
    private double mLastAngle; 
    private double mDesiredAngle; 
    private double mLastSpeed; 
 
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA); 
 
 
    /* CONSTRUCTOR */ 
 
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){ 
        this.moduleNumber = moduleNumber; 

        configAngleMotor(moduleConstants.angleMotorID);
        configDriveMotor(moduleConstants.driveMotorID);

        mLastAngle = getState().angle.getDegrees(); 
    } 

    public void voltageDrive(double v){
        anglePIDController.setReference(0, ControlType.kPosition);
        mDriveMotor.setVoltage(v);
    }

    public void sysIdLog(SysIdRoutineLog log){
        log.motor("m"+moduleNumber).voltage(
            edu.wpi.first.units.Units.Volts.of(mDriveMotor.getAppliedOutput() * RobotController.getBatteryVoltage())
            ).linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(mDriveEncoder.getVelocity()))
            .linearPosition(edu.wpi.first.units.Units.Meters.of(mDriveMotor.getEncoder().getPosition()));
    }

 
 
    /* METHODS */
 
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) 
    { 
        desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
 
        if(isOpenLoop){ // TELEOP 
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed; 
            mDriveMotor.set(percentOutput * -1); // TODO remove when inverting works
        } 
        else{ // AUTO 
            double velocity = Conversions.MPStoRPM(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio); //TODO update for neos? 
            drivePIDController.setReference(velocity * -1, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond * -1)); // TODO fix when inverts work
        } 
 
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? mLastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less than 1%. Prevents jittering. 
        anglePIDController.setReference(angle+180, ControlType.kPosition); 
        mDesiredAngle = angle; 
        mLastAngle = angle;
        SmartDashboard.putNumber("swerve/mod"+moduleNumber+"/velocitySetpointMPS", desiredState.speedMetersPerSecond);
        SmartDashboard.putNumber("swerve/mod"+moduleNumber+"/velocitySetpointRPM", Conversions.MPStoRPM(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio));
        SmartDashboard.putNumber("swerve/mod"+moduleNumber+"/velocity", mDriveEncoder.getVelocity());
    } 
 
 
    /* CURRENT STATE */ 
 
    public SwerveModuleState getState(){ 
        double velocity = mDriveEncoder.getVelocity() * -1; //Units configured to m/s TODO change when inverts work
        Rotation2d angle = getAngle(); 
        return new SwerveModuleState(velocity, angle); 
    } 
 
    public SwerveModulePosition getPosition(){ 
        double distance = mDriveEncoder.getPosition() * -1; //Units configured to m TODO change when inverts work
        Rotation2d angle = getAngle(); 
        return new SwerveModulePosition(distance, angle); 
    } 
 
    public Rotation2d getAngle(){ 
        return Rotation2d.fromDegrees(mAngleEncoder.getPosition()-180); 
    } 
 
 
    /* OTHER ACCESSORS */ 
 
    /** 
     * Get temp of a motor in this swerve module 
     * @param motor motor index 1 is drive motor, any other number is angle motor 
     * @return 
     */ 
    public Double getTemp(int motor){ 
        return (motor == 1)?mDriveMotor.getMotorTemperature():mAngleMotor.getMotorTemperature(); 
    } 
 
    public double getDesiredAngle(){ 
        return mDesiredAngle; 
    } 
 
    public double getDesiredSpeed(){ 
        return mLastSpeed; 
    }


    /* CONFIGURATION */
    private void configAngleMotor(int id){
        mAngleMotor = new CANSparkMax(id, MotorType.kBrushless);
        SparkUtilities.optimizeFrames(mAngleMotor, false, false, false, false, false, true);
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setSmartCurrentLimit(30);

        mAngleEncoder = mAngleMotor.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle); 
        mAngleEncoder.setPositionConversionFactor(360); 
        mAngleEncoder.setInverted(Constants.Swerve.canCoderInvert);

        anglePIDController = mAngleMotor.getPIDController();
        anglePIDController.setFeedbackDevice(mAngleEncoder); 
        anglePIDController.setPositionPIDWrappingMinInput(0); 
        anglePIDController.setPositionPIDWrappingMaxInput(360); 
        anglePIDController.setPositionPIDWrappingEnabled(true); 
        anglePIDController.setP(Constants.Swerve.angleKP); 
        anglePIDController.setI(Constants.Swerve.angleKI); 
        anglePIDController.setD(Constants.Swerve.angleKD);

        /*
         * TODO I know why inverting the encoder fixed it. Because for some reason the encoder invert
         * was in the motor invert so inverting the encoder ACTUALLY inverted the motor :/
         */
    }
    private void configDriveMotor(int id){
        /* Drive Motor Config */ 
        mDriveMotor = new CANSparkFlex(id, MotorType.kBrushless); 
        SparkUtilities.optimizeFrames(mDriveMotor, false, true, true, false, false, false); 
        mDriveMotor.setInverted(false); // TODO Setting this to true breaks it. Manually invert motor sets and encoder reads
        mDriveMotor.setSmartCurrentLimit(60);
        //configDriveMotor();

        /* Drive Encoder Config */
        mDriveEncoder = mDriveMotor.getEncoder(); 
        // Set to m/s for speed and m for distance 
        mDriveEncoder.setPositionConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio); 
        mDriveEncoder.setVelocityConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio / 60.0); 

        drivePIDController = mDriveMotor.getPIDController();
        drivePIDController.setFeedbackDevice(mDriveEncoder);
        drivePIDController.setP(Constants.Swerve.driveKP);
        drivePIDController.setI(Constants.Swerve.angleKI);
        drivePIDController.setD(Constants.Swerve.driveKD);
    }
 
 
    /* LOGGING AND SAVING */ 
 
    public void sendTelemetry(){ 
        //LogOrDash.logNumber("swerve/m" + moduleNumber + "/cancoder", getAngle().getDegrees()); 
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/position", getState().angle.getDegrees()); 
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/velocity", getState().speedMetersPerSecond); 
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/velocity", getPosition().distanceMeters); 
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/angle/setpoint", mDesiredAngle); 
        LogOrDash.logNumber("swerve/m" + moduleNumber + "/drive/setpoint", mLastSpeed); 
         
        LogOrDash.sparkMaxDiagnostics("swerve/m" + moduleNumber + "/angle", mAngleMotor); 
        LogOrDash.sparkMaxDiagnostics("swerve/m" + moduleNumber + "/drive", mDriveMotor); 
 
        LogOrDash.logNumber("swerve/m"+moduleNumber+"/angle/raw_analog", mAngleEncoder.getPosition()); 
    } 
 
    /** 
     * Set settings for this motor controller and save tham to its flash memory. 
     *  
     * This is only intended to be done when hardware is replace or settings changed, 
     * NOT on each boot! This prevents failed configuration or carryover from previous code. 
     */ 
    public void configToFlash() 
    { 
        try 
        { 
            // Drive motor 
            LogOrDash.checkRevError("drive motor "+moduleNumber+" clear", 
                mDriveMotor.restoreFactoryDefaults()); 
             
            Thread.sleep(1000); 
 
            SparkPIDController pid = mDriveMotor.getPIDController(); 
 
            LogOrDash.checkRevError("drive motor "+moduleNumber+" kp", 
                pid.setP(Constants.Swerve.driveKP)); 
            LogOrDash.checkRevError("drive motor "+moduleNumber+" ki", 
                pid.setI(Constants.Swerve.driveKI)); 
            LogOrDash.checkRevError("drive motor "+moduleNumber+" kd", 
                pid.setD(Constants.Swerve.driveKD)); 
            LogOrDash.checkRevError("drive motor "+moduleNumber+" kf", 
                pid.setFF(Constants.Swerve.driveKF)); 
                 
            LogOrDash.checkRevError("drive motor "+moduleNumber+" open loop ramp", 
                mDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp)); 
 
            LogOrDash.checkRevError("drive motor "+moduleNumber+" closed loop ramp", 
                mDriveMotor.setOpenLoopRampRate(Constants.Swerve.closedLoopRamp)); 
             
            LogOrDash.checkRevError("drive motor "+moduleNumber+" current", 
                mDriveMotor.setSmartCurrentLimit(Constants.Swerve.drivePeakCurrentLimit, Constants.Swerve.driveContinuousCurrentLimit)); 
 
            LogOrDash.checkRevError("drive motor "+moduleNumber+" idle mode", 
                mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode)); 
 
            // This doesn't return a RevLibError apparently 
            mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert); 
 
            Thread.sleep(1000); 
            LogOrDash.checkRevError("drive motor "+moduleNumber+" BURN", 
                mDriveMotor.burnFlash()); 
            Thread.sleep(1000); 
 
            // Anale motor 
            LogOrDash.checkRevError("angle motor "+moduleNumber+" clear", 
                mAngleMotor.restoreFactoryDefaults()); 
             
            Thread.sleep(1000); 
 
            pid = mAngleMotor.getPIDController(); 
 
            LogOrDash.checkRevError("angle motor "+moduleNumber+" kp", 
                pid.setP(Constants.Swerve.angleKP)); 
            LogOrDash.checkRevError("angle motor "+moduleNumber+" ki", 
                pid.setI(Constants.Swerve.angleKI)); 
            LogOrDash.checkRevError("angle motor "+moduleNumber+" kd", 
                pid.setD(Constants.Swerve.angleKD)); 
            LogOrDash.checkRevError("angle motor "+moduleNumber+" kf", 
                pid.setFF(Constants.Swerve.angleKF)); 
             
            LogOrDash.checkRevError("angle motor "+moduleNumber+" current", 
                mAngleMotor.setSmartCurrentLimit(Constants.Swerve.anglePeakCurrentLimit, Constants.Swerve.angleContinuousCurrentLimit)); 
 
            LogOrDash.checkRevError("angle motor "+moduleNumber+" idle mode", 
                mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode)); 
 
            // This doesn't return a RevLibError apparently 
            mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert); 
  
            Thread.sleep(1000); 
            LogOrDash.checkRevError("angle motor "+moduleNumber+" BURN", 
                mAngleMotor.burnFlash()); 
            Thread.sleep(1000); 
        } 
        catch(InterruptedException e) 
        { 
            DriverStation.reportError("Main thread interrupted while flashing swerve module!", e.getStackTrace()); 
        } 
    } 


    public void burnFlash(){
        try{
          Thread.sleep(1000);
          mDriveMotor.burnFlash();
          Thread.sleep(1000);
          mAngleMotor.burnFlash();
          Thread.sleep(1000);
        }catch(InterruptedException e){
          DriverStation.reportError("Thread was interrupted while flashing swerve module", e.getStackTrace());
        }
      }

    public double getAngleCurrent(){
        return mAngleMotor.getOutputCurrent();
    }
    public double getDriveCurrent(){
        return mDriveMotor.getOutputCurrent();
    }
} 