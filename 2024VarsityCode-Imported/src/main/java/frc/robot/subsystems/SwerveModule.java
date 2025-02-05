package frc.robot.subsystems; 
 
import com.revrobotics.RelativeEncoder; 
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.lib.math.Conversions; 
import frc.lib.util.SwerveModuleConstants;
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
    private SparkMax mAngleMotor;
    private SparkFlex mDriveMotor;
 
    private RelativeEncoder mDriveEncoder; 
    private AbsoluteEncoder mAngleEncoder;
 
    private double mLastAngle; 
    private double mDesiredAngle; 
    private double mLastSpeed; 

    private static final SparkFlexConfig DRIVE_CONFIG = new SparkFlexConfig();
    private static final SparkMaxConfig ANGLE_CONFIG = new SparkMaxConfig();
 
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA); 
 
 
    /* CONSTRUCTOR */ 
 
    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){ 
        this.moduleNumber = moduleNumber; 




        /* ANGLE MOTOR CONFIGURATION */
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        ANGLE_CONFIG
            .smartCurrentLimit(Constants.Swerve.angleCurrentLimit)
            .inverted(Constants.Swerve.angleMotorInvert)
            .idleMode(IdleMode.kBrake);

        ANGLE_CONFIG.closedLoop
            .p(Constants.Swerve.angleKP)
            .i(Constants.Swerve.angleKI)
            .d(Constants.Swerve.angleKD)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .velocityFF(Constants.Swerve.angleKF)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 360);

        ANGLE_CONFIG.absoluteEncoder
            .inverted(Constants.Swerve.angleMotorInvert)
            .positionConversionFactor(360);

        SparkMaxConfig motorConfig = new SparkMaxConfig();
        AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
        ClosedLoopConfig controllerConfig = new ClosedLoopConfig();

        mAngleMotor.configure(ANGLE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        /* DRIVE MOTOR CONFIGURATION */
        mDriveMotor = new SparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        DRIVE_CONFIG
            .closedLoopRampRate(Constants.Swerve.closedLoopRamp)
            .openLoopRampRate(Constants.Swerve.openLoopRamp)
            .smartCurrentLimit(Constants.Swerve.driveCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .inverted(Constants.Swerve.driveMotorInvert);

        DRIVE_CONFIG.encoder
            .positionConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio)
            .velocityConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio / 60.0);

        DRIVE_CONFIG.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Constants.Swerve.driveKP)
            .i(Constants.Swerve.driveKI)
            .d(Constants.Swerve.driveKD)
            .velocityFF(Constants.Swerve.driveKF);

        mDriveMotor.configure(DRIVE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        mAngleEncoder = mAngleMotor.getAbsoluteEncoder();
        mDriveEncoder = mDriveMotor.getEncoder();
        mLastAngle = getState().angle.getDegrees(); 
    } 
 
 
    /* METHODS */
 
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) 
    { 
        desiredState.optimize(getState().angle);
 
        if(isOpenLoop){ // TELEOP 
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed; 
            mDriveMotor.set(percentOutput);
        } 
        else{ // AUTO 
            double velocity = Conversions.MPStoRPM(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward.calculate(desiredState.speedMetersPerSecond));
        } 
 
        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? mLastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less than 1%. Prevents jittering. 
        mDriveMotor.getClosedLoopController().setReference(angle+180, ControlType.kPosition); 
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
 

   
    /* SysId Testing */
    public void voltageDrive(double v){
        mDriveMotor.getClosedLoopController().setReference(0, ControlType.kPosition);
        mDriveMotor.setVoltage(v);
    }

    public void sysIdLog(SysIdRoutineLog log){
        log.motor("m"+moduleNumber).voltage(
            edu.wpi.first.units.Units.Volts.of(mDriveMotor.getAppliedOutput() * RobotController.getBatteryVoltage())
            ).linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(mDriveEncoder.getVelocity()))
            .linearPosition(edu.wpi.first.units.Units.Meters.of(mDriveMotor.getEncoder().getPosition()));
    }
} 