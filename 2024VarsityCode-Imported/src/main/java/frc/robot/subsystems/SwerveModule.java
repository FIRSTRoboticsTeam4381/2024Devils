package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

@Logged
public class SwerveModule {
    public int moduleNumber;
    private SparkMax mAngleMotor;
    private SparkFlex mDriveMotor;

    private SparkAbsoluteEncoder absoluteEncoder;

    private RelativeEncoder distanceEncoder;

    private double lastAngle;
    private double desiredAngle;
    private double lastSpeed;

    private static final SparkFlexConfig DRIVE_CONFIG = new SparkFlexConfig();
    private static final SparkMaxConfig ANGLE_CONFIG = new SparkMaxConfig();
            

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;

        /* Angle Motor Config */
        mAngleMotor = new SparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        ANGLE_CONFIG
            .smartCurrentLimit(Constants.Swerve.angleCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .inverted(Constants.Swerve.angleMotorInvert);    
            
        ANGLE_CONFIG.closedLoop
            .p(Constants.Swerve.angleKP)
            .i(Constants.Swerve.angleKI)
            .d(Constants.Swerve.angleKD)
            .velocityFF(Constants.Swerve.angleKF)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 360);

        ANGLE_CONFIG.absoluteEncoder
            .zeroOffset(mAngleMotor.configAccessor.absoluteEncoder.getZeroOffset())
            .positionConversionFactor(360);
        
        mAngleMotor.configure(ANGLE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        /* Drive Motor Config */
        mDriveMotor = new SparkFlex(moduleConstants.driveMotorID, MotorType.kBrushless);
        DRIVE_CONFIG
            .closedLoopRampRate(Constants.Swerve.closedLoopRamp)
            .openLoopRampRate(Constants.Swerve.openLoopRamp)
            .smartCurrentLimit(Constants.Swerve.driveCurrentLimit)
            .idleMode(IdleMode.kBrake)
            .inverted(Constants.Swerve.driveMotorInvert);    
            
        DRIVE_CONFIG.closedLoop
            .p(Constants.Swerve.driveKP)
            .i(Constants.Swerve.driveKI)
            .d(Constants.Swerve.driveKD)
            .velocityFF(Constants.Swerve.driveKF);

        DRIVE_CONFIG.encoder
            .positionConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio)
            .velocityConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio / 60.0);
            
        
        mDriveMotor.configure(DRIVE_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        

        /* Angle Encoder Config */
        absoluteEncoder = mAngleMotor.getAbsoluteEncoder();
        //absoluteEncoder.setPositionConversionFactor(360); Now set in config and saved to memory
        //absoluteEncoder.setZeroOffset(-180);

        //mAngleMotor.getPIDController().setFeedbackDevice(absoluteEncoder);
        //mAngleMotor.getPIDController().setPositionPIDWrappingMinInput(-180);
        //mAngleMotor.getPIDController().setPositionPIDWrappingMaxInput(180);
        //mAngleMotor.getPIDController().setPositionPIDWrappingEnabled(true);

        distanceEncoder = mDriveMotor.getEncoder();
        // Set to m/s for speed and m for distance
        //distanceEncoder.setPositionConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio);
        //distanceEncoder.setVelocityConversionFactor(Constants.Swerve.wheelCircumference / Constants.Swerve.driveGearRatio / 60.0);

        lastAngle = getState().angle.getDegrees();
        
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop)
    {
        desiredState.optimize(getState().angle);

         if(isOpenLoop){ // TELEOP 
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed; 
            mDriveMotor.set(percentOutput); // TODO remove when inverting works
        } 
        else{ // AUTO 
            double velocity = Conversions.MPStoRPM(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio); //TODO update for neos? 
            mDriveMotor.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedforward.calculate(desiredState.speedMetersPerSecond)); // TODO fix when inverts work
        } 

        //double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less than 1%. Prevents jittering.
        double angle = desiredState.angle.getDegrees();
        mAngleMotor.getClosedLoopController().setReference(angle+180, ControlType.kPosition);
        desiredAngle = angle;
        lastAngle = angle;
    }
    
    
    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(absoluteEncoder.getPosition()-180);
    }

    /**
     * Get temp of a motor in this swerve module
     * @param motor motor index 1 is drive motor, any other number is angle motor
     * @return
     */
    public Double getTemp(int motor){
        return (motor == 1)?mDriveMotor.getMotorTemperature():mAngleMotor.getMotorTemperature();
    }

    public double getDesiredAngle(){
        return desiredAngle;
    }

    public double getDesiredSpeed(){
        return lastSpeed;
    }

    public SwerveModuleState getState(){
        double velocity = distanceEncoder.getVelocity() * driveInvert(); //Units configured to m/s
        Rotation2d angle = getAngle();
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition(){
        double distance = distanceEncoder.getPosition() * driveInvert(); //Units configured to m
        Rotation2d angle = getAngle();
        return new SwerveModulePosition(distance, angle);
    }

    public void sendTelemetry(){
        //LogOrDash.logNumber("swerve/m" + moduleNumber + "/cancoder", getAngle().getDegrees());
        /*
        None of this should be needed w/ new auto logging
        SmartDashboard.putNumber("swerve/m" + moduleNumber + "/angle/position", getState().angle.getDegrees());
        SmartDashboard.putNumber("swerve/m" + moduleNumber + "/drive/velocity", getState().speedMetersPerSecond);
        SmartDashboard.putNumber("swerve/m" + moduleNumber + "/drive/position", getPosition().distanceMeters);
        SmartDashboard.putNumber("swerve/m" + moduleNumber + "/angle/setpoint", desiredAngle);
        SmartDashboard.putNumber("swerve/m" + moduleNumber + "/drive/setpoint", lastSpeed);
        
       
        SmartDashboard.putNumber("swerve/m"+moduleNumber+"/angle/raw_analog", absoluteEncoder.getPosition());*/
    }


    // Stuff for SysID drivetrain tests
    public void setVoltage(double v)
    {
        mAngleMotor.getClosedLoopController().setReference(0, ControlType.kPosition);
        mDriveMotor.setVoltage(v);
    }

    public void logSysIDData(SysIdRoutineLog log)
    {
        log.motor("m"+moduleNumber).voltage(
            edu.wpi.first.units.Units.Volts.of(mDriveMotor.getAppliedOutput() * RobotController.getBatteryVoltage())
            ).linearVelocity(edu.wpi.first.units.Units.MetersPerSecond.of(mDriveMotor.getEncoder().getVelocity()))
            .linearPosition(edu.wpi.first.units.Units.Meters.of(mDriveMotor.getEncoder().getPosition()));
    }

    public int driveInvert()
    {
        if(Constants.Swerve.driveMotorInvert)
            return -1;
        else
            return 1;
    }

    
}
