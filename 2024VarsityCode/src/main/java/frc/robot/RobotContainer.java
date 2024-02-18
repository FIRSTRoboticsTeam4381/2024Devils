// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.LogOrDash;
//import frc.robot.autos.Autos;
import frc.robot.commands.ClimbDefault;
import frc.robot.commands.ShooterPivotDefault;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPivot;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final CommandPS4Controller specialist = new CommandPS4Controller(1);
    private final CommandPS4Controller driver = new CommandPS4Controller(0);

    /* Driver Buttons */
    private final Trigger zeroSwerve = driver.options();

    /* Operator Buttons */
    private final Trigger intake = specialist.cross();
    private final Trigger eject = specialist.circle();
    private final Trigger shoot = specialist.R1();
    private final Trigger feedNote = specialist.L1();
    private final Trigger ejectTop = specialist.triangle();
    private final Supplier<Double> leftYAxis = specialist::getLeftY;
    private final Supplier<Double> rightYAxis = specialist::getRightY;
    private final Supplier<Double> r2Axis = specialist::getR2Axis;
    private final Supplier<Double> l2Axis = specialist::getL2Axis;

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Intake s_Intake = new Intake();
    public static final Shooter s_Shooter = new Shooter();
    public static final Indexer s_Index = new Indexer(s_Intake, s_Shooter);
    public static final ShooterPivot s_Pivot = new ShooterPivot();
    public static final Climb s_Climb = new Climb();

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer(){
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, true));
        s_Pivot.setDefaultCommand(new ShooterPivotDefault(s_Pivot, r2Axis, l2Axis));
        s_Climb.setDefaultCommand(new ClimbDefault(s_Climb, leftYAxis, rightYAxis));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
    private void configureButtonBindings(){
        // Button to reset swerve odometry and angle
        zeroSwerve.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));

        intake.whileTrue(s_Index.intake());
        eject.onTrue(s_Index.startEject()).onFalse(s_Index.stopIntake());

        shoot.onTrue(s_Shooter.startShooter()).onFalse(s_Shooter.stopShooter());
        ejectTop.onTrue(s_Shooter.startTopEject()).onFalse(s_Shooter.stopShooter());
        feedNote.whileTrue(s_Index.feedNote());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(){
        return Commands.none();
    }
}