// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.commands.ComposedCommands;
import frc.robot.commands.ManualClimb;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private static final CommandPS4Controller driver = new CommandPS4Controller(0);
    private static final CommandPS4Controller specialist = new CommandPS4Controller(1);

    /* Subsystems */
    public static final Swerve s_Swerve = new Swerve();
    public static final Intake s_Intake = new Intake();
    public static final Index s_Index = new Index();
    public static final Pivot s_Pivot = new Pivot();
    public static final Shooter s_Shooter = new Shooter();
    public static final Climb s_Climb = new Climb();
    public static final Limelight s_LL = new Limelight();
    //public static final LEDs s_LED = new LEDs();

    /* Commands */
   public static final ComposedCommands commands = new ComposedCommands(specialist, s_Intake, s_Index, s_Shooter, s_Pivot, s_Climb, s_LL, s_Swerve);

    // Auto Chooser
    SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer(){
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver::getLeftY, driver::getLeftX, driver::getRightX, true, driver::L1).withName("Teleop"));
        s_Pivot.setDefaultCommand(new ManualPivot(specialist::getLeftY, s_Pivot).withName("Manual Pivot"));
        s_Climb.setDefaultCommand(new ManualClimb(specialist, s_Climb));

        SmartDashboard.putData("PDP", new PowerDistribution());
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        // Configure the button bindings
        configureButtonBindings();

        // LED Status Effects
        //s_LED.clear();
        //CommandScheduler.getInstance().schedule(s_LED.noteStoredConditional());
    }

    /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
    private void configureButtonBindings(){
        // Button to reset swerve odometry and angle
        driver.options()
            .onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())
            .alongWith(new InstantCommand(() -> s_Swerve.resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0))))));
        // Shoot Note
        driver.R1().or(specialist.R1()).whileTrue(commands.feedNote());
        //driver.back().onTrue(new InstantCommand(()->s_LL.takeSnapshot())).onFalse(new InstantCommand(()->s_LL.resetSnapshot()));
        //driver.back().onTrue(commands.climb());

        specialist.square().toggleOnTrue(commands.subwooferMode());
        specialist.cross().toggleOnTrue(commands.groundIntake(new ManualPivot(specialist::getLeftY, s_Pivot)));
        specialist.circle().whileTrue(commands.ejectNote());

        specialist.povRight().toggleOnTrue(commands.ampMode());
        specialist.povDown().whileTrue(commands.reverseAmp());
        specialist.povLeft().toggleOnTrue(s_Shooter.trapShoot());
        specialist.povUp().toggleOnTrue(commands.podiumMode());

        specialist.L1().toggleOnTrue(commands.startShooter());
        //specialist.back().toggleOnTrue(commands.allianceLineMode());

        driver.touchpad().or(specialist.touchpad()).onTrue(commands.cancelAll());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(){
        return m_AutoChooser.getSelected();
    }

      
  // Static reference to the robot class
  // Previously we used static subsystems, but this appears to break things in 2025
  // Use getRobot() to get the robot object
  private static RobotContainer robotReference;

  /**
   * Get a reference to the RobotContainer object in use
   * @return the active RobotContainer object
   */
  public static RobotContainer getRobot()
  {
    return robotReference;
  }
}