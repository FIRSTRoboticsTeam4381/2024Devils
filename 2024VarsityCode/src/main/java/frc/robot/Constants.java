// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double stickDeadband = 0.1;

    //public static final int NEO_TICKS_PER_REV = 42; //TODO

    public static final class Intake{
        public static final int primaryCAN = 45; // TODO
        public static final int helperCAN = 46; // TODO
    }
    public static final class Index{
        public static final int indexCAN = 47;
        public static final int indexDIO = 0;
    }
    public static final class Shooter{
        public static final int propMotorCAN = 48;
        public static final int topMotorCAN = 49;
        public static final int bottomMotorCAN = 50;
    }
    public static final class Pivot{
        public static final int rightPivotCAN = 51;
        public static final int leftPivotCAN = 52;
    }
    public static final class Climb{
        public static final int rightClimbCAN = 53;
        public static final int leftClimbCAN = 54;
        public static final int midClimbCAN = 55;
    }

    public static final class Swerve{
        public static final int pigeonID = 1; // Using navx instead
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.25);
        public static final double wheelBase = Units.inchesToMeters(21.25);
        public static final double wheelDiameter = Units.inchesToMeters(4);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (425.0 / 72.0);
        public static final double angleGearRatio = (150.0/7.0 / 1.0);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        
        // TODO check - auto - PIDs need to be configured
        public static final HolonomicPathFollowerConfig holonomicConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0), // TODO Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // TODO Rotation PID constants
            6, // Max module speed, in m/s
            0.4, // TODO Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        );

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int anglePeakCurrentLimit = 400;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.005;//0.06;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;//0.25;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.000005;//3.1534E-05; //From SysID, may not be converted right!
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.018231;//(0.667 / 12); // divide by 122 to convert from volts to percent output for CTRE TODO?
        public static final double driveKV = 1.9676;//(2.44 / 12);
        public static final double driveKA = 0.68184;//(0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 6; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kCoast;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0{
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1{
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 21;
            public static final int canCoderID = 22;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3{
            public static final int driveMotorID = 30;
            public static final int angleMotorID = 31;
            public static final int canCoderID = 32;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Left Module - Module 4 */
        public static final class Mod2{
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 41;
            public static final int canCoderID = 42;
            public static final double angleOffset = 0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants{
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = 2*Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 2*Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
    }
}
