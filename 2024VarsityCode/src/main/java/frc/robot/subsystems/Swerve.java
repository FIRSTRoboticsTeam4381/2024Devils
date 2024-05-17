package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.util.DriftCorrection;
import frc.lib.util.LogOrDash;
import frc.robot.Constants;

public class Swerve extends SubsystemBase{
    public SwerveDriveOdometry mSwerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;
    public Field2d mField;

    private SysIdRoutine routine;

    public Swerve(){
        gyro = new AHRS(Port.kUSB);
        zeroGyro();

        mField = new Field2d();

        mSwerveMods = new SwerveModule[]{
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        mSwerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getPositions());

        // TODO check - auto
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            Constants.Swerve.holonomicConfig,
            () -> {return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;},
            this // Reference to this subsystem to set requirements
        );

        routine = setupSysId();
    }

    private SysIdRoutine setupSysId(){
        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
                (volts) -> {
                    for(int i = 0; i < mSwerveMods.length; i++){
                        mSwerveMods[i].voltageDrive(volts.in(edu.wpi.first.units.Units.Volts));
                    }
                }, 
                (log) -> {
                    for(int i = 0; i < mSwerveMods.length; i++){
                        mSwerveMods[i].sysIdLog(log);
                    }
                }, 
                this)
                );
        return routine;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return routine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return routine.dynamic(direction);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){ 
        SwerveModuleState[] swerveModuleStates =  
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(DriftCorrection.driftCorrection( 
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds( 
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getYaw() 
                ) 
                : new ChassisSpeeds( 
                    translation.getX(), 
                    translation.getY(), 
                    rotation),
                mSwerveOdometry.getPoseMeters(), 
                gyro));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed); 
 
        for(SwerveModule mod : mSwerveMods){ 
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop); 
        } 
    } 

    // TODO check - auto
    public void drive(ChassisSpeeds robotRelativeSpeeds){
        Translation2d translation = new Translation2d(robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond);
        double rotation = robotRelativeSpeeds.omegaRadiansPerSecond;
        drive(translation, rotation, false, false);
    }

    // TODO check - auto
    /* Used by PathPlanner AutoBuilder */
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(
            mSwerveMods[0].getState(),
            mSwerveMods[1].getState(),
            mSwerveMods[2].getState(),
            mSwerveMods[3].getState()
        );
    }

    /*
    public double odometryDistanceFromGoal(){
        Alliance alliance = DriverStation.getAlliance().get();
        double[] redGoal = {16, 5.5, 0};
        double[] blueGoal = {0, 5.5, 0};
        Pose2d robotPose = getPose();
        double distance = Math.sqrt(Math.pow(robotPose.getX()-(alliance==Alliance.Red?redGoal[0]:blueGoal[0]),2) + Math.pow(robotPose.getY()-(alliance==Alliance.Red?redGoal[1]:blueGoal[1]),2));
        return distance;
    }
    */

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /**
     * @return XY of robot on field
     */
    public Pose2d getPose(){
        return mSwerveOdometry.getPoseMeters();
    }

    /**
     * Use to reset odometry to a certain known pose or to zero
     * @param pose Desired new pose
     */
    public void resetOdometry(Pose2d pose){
        mSwerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public void resetOdometry(Pose2d pose, Rotation2d yaw){
        mSwerveOdometry.resetPosition(yaw, getPositions(), pose);
    }

    public void resetOdometry(Rotation2d yaw){
        mSwerveOdometry.resetPosition(yaw, getPositions(), mSwerveOdometry.getPoseMeters());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * @return Swerve Module positions
     */
    public SwerveModulePosition[] getPositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Use to reset angle to certain known angle or to zero
     * @param angle Desired new angle
     */
    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw(){
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    @Override
    public void periodic(){
        mSwerveOdometry.update(getYaw(), getPositions());
        SmartDashboard.putBoolean("swerve/Gyro Calibrated", !gyro.isCalibrating());

        LogOrDash.logNumber("Gyro Angle", getYaw().getDegrees());

        SwerveModuleState[] currentStatus = new SwerveModuleState[4];
        double[] targetSpeeds = new double[4];
        double[] targetAngles = new double[4];
        double[] absoluteAngles = new double[4];

        SmartDashboard.putNumber("swerve/rotation", getYaw().getDegrees()-180);
        SmartDashboard.putNumber("match/Match Timer", DriverStation.getMatchTime());

        for(SwerveModule m : mSwerveMods){
            SmartDashboard.putNumber("swerve/mod"+m.moduleNumber+"/Angle Current", m.getAngleCurrent());
            SmartDashboard.putNumber("swerve/mod"+m.moduleNumber+"/Drive Current", m.getDriveCurrent());
        }
        
        for(SwerveModule mod : mSwerveMods){
            mod.sendTelemetry();
            currentStatus[mod.moduleNumber] = mod.getState();
            targetSpeeds[mod.moduleNumber] = mod.getDesiredSpeed();
            targetAngles[mod.moduleNumber] = mod.getDesiredAngle();
            absoluteAngles[mod.moduleNumber] = mod.getAngle().getDegrees();
        }


        // Compile swerve status for AdvantageScope
        double[] targetStateAdv = new double[8];
        double[] currentStateAdv = new double[8];
        double[] absoluteStateAdv = new double[8];
        for(int i=0; i<4;i++)
        {
            targetStateAdv[2*i] = targetAngles[i];
            targetStateAdv[2*i+1] = targetSpeeds[i];
            
            currentStateAdv[2*i] = currentStatus[i].angle.getDegrees();
            currentStateAdv[2*i+1] = currentStatus[i].speedMetersPerSecond;

            absoluteStateAdv[2*i] = absoluteAngles[i];
            absoluteStateAdv[2*i+1] = 8;//Arbitrary to make these easier to see
        }

        SmartDashboard.putNumberArray("swerve/status", currentStateAdv);
        SmartDashboard.putNumberArray("swerve/target", targetStateAdv);
        SmartDashboard.putNumberArray("swerve/absolute", absoluteStateAdv);
        
        //SmartDashboard.putNumber("position/Distance From Goal (Odometry)", odometryDistanceFromGoal());

        mField.setRobotPose(getPose());
        SmartDashboard.putData("position/Field", mField);


        LogOrDash.logNumber("Gyro Pitch", gyro.getPitch());
        LogOrDash.logNumber("Gyro Roll", gyro.getRoll());
        LogOrDash.logNumber("Gyro Yaw", gyro.getYaw());
        LogOrDash.logString("XY Coord", "(" + getPose().getX() + ", " + getPose().getY() + ")");
    }

    public Command configToFlash(){
        return new InstantCommand(() -> {
            for(SwerveModule mod : mSwerveMods)
            {
                mod.burnFlash();
            }
        }, this).ignoringDisable(true);
    }

    private void setDriveCurrentLimit(int limit){
        for(SwerveModule m : mSwerveMods){
            m.setDriveCurrentLimit(limit);
        }
    }
    public Command nitro(){
        return new SequentialCommandGroup(
            new InstantCommand(()->setDriveCurrentLimit(80)),
            new WaitCommand(5),
            new InstantCommand(()->setDriveCurrentLimit(60))
        );
    }
    public void setBrakeMode(boolean enabled){
        for(SwerveModule m : mSwerveMods){
            m.setBrakeMode(enabled);
        }
    }
    public void addPoseToField(String label, Pose2d pose){
        mField.getObject(label).setPose(pose);
    }
}
