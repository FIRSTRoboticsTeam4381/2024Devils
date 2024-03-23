// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.lib.util.LogOrDash;
import frc.robot.autos.Autos;
import frc.robot.commands.AutoRotatingSwerve;
import frc.robot.commands.ComposedCommands;
import frc.robot.commands.ManualClimb;
import frc.robot.commands.ManualPivot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// TODO climb positions for auto climbing
// TODO flip pivot encoder
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
    public static final LEDs s_LED = new LEDs();

    /* Commands */
   public static final ComposedCommands commands = new ComposedCommands(specialist, s_Intake, s_Index, s_Shooter, s_Pivot, s_LL, s_Swerve, s_LED);

    // Auto Chooser
    SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer(){
        s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, true).withName("Teleop"));
        s_Pivot.setDefaultCommand(new ManualPivot(specialist::getLeftY, s_Pivot).withName("Manual Pivot"));

        SmartDashboard.putData("PDP", new PowerDistribution());
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        // Configure the button bindings
        configureButtonBindings();
        

        /* Pathplanner Commands */
        registerCommands();

        /* Burn Flash Buttons */
        configMotorSettingButtons();

        /* Autonomous Chooser */
        setupAutoOptions();
    }

    private void setupAutoOptions(){
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {s_Swerve.mField.setRobotPose(pose);});
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {s_Swerve.mField.getObject("target pose").setPose(pose);});
        PathPlannerLogging.setLogActivePathCallback((poses) -> {s_Swerve.mField.getObject("path").setPoses(poses);});

        m_AutoChooser.setDefaultOption("None", Autos.none());
        m_AutoChooser.addOption("Start2ThreePiece", Autos.start2ThreePiece());
        m_AutoChooser.addOption("StartAmp4Piece", Autos.startAmp4Piece());
        m_AutoChooser.addOption("Start3Middle", Autos.start3Middle());
        m_AutoChooser.addOption("Start4ThreePiece", Autos.start4ThreePiece());
        m_AutoChooser.addOption("Start1ThreePiece", Autos.start1ThreePiece());
        //m_AutoChooser.addOption("SysId Quas Fwd", s_Swerve.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        //m_AutoChooser.addOption("SysId Quas Rev", s_Swerve.sysIdQuasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
        //m_AutoChooser.addOption("SysId Dyna Fwd", s_Swerve.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        //m_AutoChooser.addOption("SysId Dyna Rev", s_Swerve.sysIdDynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
        SmartDashboard.putData(m_AutoChooser);
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
        
        // Auto Rotation
        driver.triangle().whileTrue(new AutoRotatingSwerve(s_Swerve, s_LL, driver, true).withName("Teleop Auto Rotate"));
        // Shoot Note
        driver.R1().or(specialist.R1()).whileTrue(commands.feedNote());
        driver.PS().onTrue(new InstantCommand(()->s_LL.takeSnapshot())).onFalse(new InstantCommand(()->s_LL.resetSnapshot()));
        driver.L1().onTrue(s_Swerve.nitro());
        driver.cross().onTrue(new InstantCommand(()->s_Swerve.setBrakeMode(true))).onFalse(new InstantCommand(()->s_Swerve.setBrakeMode(false)));

        specialist.square().toggleOnTrue(commands.humanIntake());
        specialist.cross().toggleOnTrue(commands.groundIntake(new ManualPivot(specialist::getLeftY, s_Pivot)));
        specialist.circle().whileTrue(commands.ejectNote());
        specialist.triangle().whileTrue(commands.autoAim());

        specialist.povRight().onTrue(commands.ampMode());
        specialist.povDown().whileTrue(new InstantCommand(()->s_Shooter.setCurrentLimit(80,80))
                                        .andThen(commands.reverseAmp()))
                            .onFalse(new InstantCommand(()->s_Shooter.setCurrentLimit(60, 60)));
        specialist.povLeft().toggleOnTrue(s_Shooter.ampShoot());
        specialist.povUp().onTrue(s_Pivot.goToAngle(90));

        specialist.L1().toggleOnTrue(commands.startShooter());
        specialist.PS().toggleOnTrue(new ManualClimb(specialist, s_Climb));

        driver.touchpad().or(specialist.touchpad()).onTrue(commands.cancelAll());
    }

    private void registerCommands(){
        NamedCommands.registerCommand("Intake", commands.groundIntake(commands.autoAim()));
        NamedCommands.registerCommand("StopIntake", s_Intake.instantStop());
        NamedCommands.registerCommand("ShooterSpinUp", s_Shooter.instantSetVelocityReference(Shooter.avgRPM, false));
        NamedCommands.registerCommand("AutoAim", commands.autoAim());
        NamedCommands.registerCommand("Shoot", s_Index.run());
        NamedCommands.registerCommand("LowerPivot", s_Pivot.goToAngle(25.0));
    }

    private void configMotorSettingButtons(){
        // Button to turn on/off sending debug data to the dashboard
        SmartDashboard.putData("Toggle Debug Dashboards", LogOrDash.toggleDashboard());
        //SmartDashboard.putData("Burn Spark Settings", s_Swerve.configToFlash());
        SmartDashboard.putData("configs/Burn Intake Settings", new InstantCommand(() -> s_Intake.burnFlash()));
        SmartDashboard.putData("configs/Burn Index Settings", new InstantCommand(() -> s_Index.burnFlash()));
        SmartDashboard.putData("configs/Burn Shooter Settings", new InstantCommand(() -> s_Shooter.burnFlash()));
        SmartDashboard.putData("configs/Burn Pivot Settings", new InstantCommand(() -> s_Pivot.burnFlash()));
        //SmartDashboard.putData("configs/Burn Climb Settings", new InstantCommand(() -> s_Climb.burnFlash()));
        SmartDashboard.putData("configs/Burn Swerve Settings", new InstantCommand(() -> s_Swerve.configToFlash()));

        SmartDashboard.putData("shooter/Reset PIDs", new InstantCommand(() -> s_Shooter.resetPID()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     * 
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand(){
        return m_AutoChooser.getSelected();
    }
}