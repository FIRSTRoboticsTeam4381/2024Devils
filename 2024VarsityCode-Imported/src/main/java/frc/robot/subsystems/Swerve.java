package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;

@Logged
public class Swerve extends SubsystemBase{
    public SwerveDrivePoseEstimator swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    // Secondary references for automated logging
    private SwerveModule FL;
    private SwerveModule FR;
    private SwerveModule BL;
    private SwerveModule BR;

    StructArrayPublisher<SwerveModuleState> modStatusPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Swerve/ModuleStatus", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> modTargetPublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Swerve/ModuleTarget", SwerveModuleState.struct).publish();

    StructPublisher<ChassisSpeeds> chasStatusPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Swerve/ChassisStatus", ChassisSpeeds.struct).publish();
    StructPublisher<ChassisSpeeds> chasTargetPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Swerve/ChassisTarget", ChassisSpeeds.struct).publish();

    public final Field2d field = new Field2d();
    private Pose2d startPose = new Pose2d(Units.inchesToMeters(177), Units.inchesToMeters(214), Rotation2d.fromDegrees(0));


    public Swerve(){
        gyro = new AHRS(NavXComType.kMXP_SPI);
        zeroGyro();

        SmartDashboard.putData(this);

        mSwerveMods = new SwerveModule[]{
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // Set references for auto logging
        FL = mSwerveMods[0];
        FR = mSwerveMods[1];
        BL = mSwerveMods[2];
        BR = mSwerveMods[3];

        swerveOdometry = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getYaw(), getPositions(), startPose);

        // Setup position logging
        SmartDashboard.putData("Field", field);
        //m_field.setRobotPose(startPose);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });


          setupSysIDTests();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
        
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(
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
            0.02
            );

        chasTargetPublisher.set(targetSpeeds);

        SwerveModuleState[] swerveModuleStates = 
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        modTargetPublisher.set(swerveModuleStates);

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
    private ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(
            mSwerveMods[0].getState(),
            mSwerveMods[1].getState(),
            mSwerveMods[2].getState(),
            mSwerveMods[3].getState()
        );
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Command brake() {
        return new RunCommand(() -> {
            setModuleStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
            });
        },this);
    }

    /**
     * @return XY of robot on field
     */
    public Pose2d getPose(){
        return swerveOdometry.getEstimatedPosition();
    }

    /**
     * Use to reset odometry to a certain known pose or to zero
     * @param pose Desired new pose
     */
    public void resetOdometry(Pose2d pose){
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
    }

    public void resetOdometry(Pose2d pose, Rotation2d yaw){
        swerveOdometry.resetPosition(yaw, getPositions(), pose);
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
        swerveOdometry.update(getYaw(), getPositions());
        
        resetToEdge();

        SmartDashboard.putNumber("Gyro Angle", getYaw().getDegrees());

        SwerveModuleState[] currentStatus = new SwerveModuleState[4];
        
        for(SwerveModule mod : mSwerveMods){
            mod.sendTelemetry();
            currentStatus[mod.moduleNumber] = mod.getState();
        }

        modStatusPublisher.set(currentStatus);
        chasStatusPublisher.set(getRobotRelativeSpeeds());


        SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Gyro Roll", gyro.getRoll());
        SmartDashboard.putNumber("Gyro Yaw", gyro.getYaw());
        
        SmartDashboard.putNumber("Gyro Yaw Velocity", gyro.getRate());

        field.setRobotPose(getPose());
    }


    private void setupSysIDTests() {

        SysIdRoutine routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                    (v) -> {
                        for(int j = 0; j < mSwerveMods.length; j++)
                        {
                            mSwerveMods[j].setVoltage(v.in(edu.wpi.first.units.Units.Volts));
                        }
                    }, 
                    (log) ->
                    {
                        for(int j = 0; j < mSwerveMods.length; j++)
                        {
                            mSwerveMods[j].logSysIDData(log);
                        }
                    }, 
                    this)
            );

            SmartDashboard.putData("SysID/drive/dyn_f", routine.dynamic(Direction.kForward));
            SmartDashboard.putData("SysID/drive/dyn_r", routine.dynamic(Direction.kReverse));
            SmartDashboard.putData("SysID/drive/quas_f", routine.quasistatic(Direction.kForward));
            SmartDashboard.putData("SysID/drive/quas_r", routine.quasistatic(Direction.kReverse));

    }

    public void resetToEdge() {
        if (getPose().getY() > 8.1) {
            swerveOdometry.resetPosition(getYaw(), getPositions(), new Pose2d(getPose().getX(), 8.1, getYaw()));        // Need to replace getPose(), getPost gets the current position, we need the desired position in the field
          } else if (getPose().getY() < -0.1) {
            swerveOdometry.resetPosition(getYaw(), getPositions(), new Pose2d(getPose().getX(), -0.1, getYaw()));      
          } if (getPose().getX() > 17.6) {
            swerveOdometry.resetPosition(getYaw(), getPositions(), new Pose2d(17.6, getPose().getY(), getYaw())); 
          } else if (getPose().getX() < -0.1) {
            swerveOdometry.resetPosition(getYaw(), getPositions(), new Pose2d(-0.1, getPose().getY(), getYaw())); 
          }
    }


}
