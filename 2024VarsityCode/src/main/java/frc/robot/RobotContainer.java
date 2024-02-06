// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.LogOrDash;
import frc.lib.util.LEDs.AlternatingTransitionEffect;
import frc.lib.util.LEDs.Colors;
import frc.lib.util.LEDs.RainEffect;
import frc.lib.util.LEDs.SolidColorEffect;
import frc.lib.util.LEDs.VisorEffect;
import frc.robot.autos.Autos;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.LEDs;
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
    private final CommandJoystick testingController = new CommandJoystick(0);

    /* Buttons */
    private final Trigger solidRed = testingController.button(3);
    private final Trigger solidWhite = testingController.button(4);
    private final Trigger blueWhiteWave = testingController.button(5);
    private final Trigger orangeVisorRedBackground = testingController.button(6);
    private final Trigger randomRainBlankBackground = testingController.button(7);
    private final Trigger whiteRainBlueBackground = testingController.button(8);
    private final Trigger blueBackgroundTwoVisors = testingController.button(9);
    private final Trigger purpleBackgroundTwoVisorsRandomRain = testingController.button(10);

    /* Subsystems */
    private static final int LED_LENGTH = 150;

    private static final int WAVE_LENGTH = 30;
    private static final int WAVE_SPEED = 5;

    private static final int VISOR_SIZE = 8;
    private static final int VISOR_SPEED = 3;

    private static final int RAIN_SPEED = 5;
    private static final int RAIN_VOLUME = 40;

    private static final LEDs leds = new LEDs(LED_LENGTH, 0);

    /** The container for the robot. Contains subsystems, IO devices, and commands. */
    public RobotContainer(){
        leds.setDefaultCommand(new LEDCommand(leds));

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
        solidRed.onTrue(new InstantCommand(() -> leds.setEffect(new SolidColorEffect(LED_LENGTH, Colors.RED))));
        solidWhite.onTrue(new InstantCommand(() -> leds.setEffect(new SolidColorEffect(LED_LENGTH, Colors.WHITE))));
        blueWhiteWave.onTrue(new InstantCommand(() -> leds.setEffect(new AlternatingTransitionEffect(LED_LENGTH, Colors.TEAL, Colors.WHITE, WAVE_LENGTH, WAVE_SPEED)))); // TODO
        orangeVisorRedBackground.onTrue(new InstantCommand(() -> leds.setEffect(new SolidColorEffect(LED_LENGTH, Colors.RED), 
                                                                                new VisorEffect(LED_LENGTH, new int[] {255, 150, 0}, VISOR_SIZE, VISOR_SPEED)))); // TODO
        randomRainBlankBackground.onTrue(new InstantCommand(() -> leds.setEffect(new RainEffect(LED_LENGTH, RAIN_SPEED, RAIN_VOLUME)))); // TODO
        whiteRainBlueBackground.onTrue(new InstantCommand(() -> leds.setEffect(new SolidColorEffect(LED_LENGTH, Colors.BLUE),
                                                                                new RainEffect(LED_LENGTH, Colors.WHITE, RAIN_SPEED, RAIN_VOLUME)))); // TODO
        blueBackgroundTwoVisors.onTrue(new InstantCommand(() -> leds.setEffect(new SolidColorEffect(LED_LENGTH, Colors.BLUE),
                                                                                new VisorEffect(LED_LENGTH, Colors.WHITE, VISOR_SIZE, VISOR_SPEED),
                                                                                new VisorEffect(LED_LENGTH, Colors.YELLOW, VISOR_SIZE, VISOR_SPEED, LED_LENGTH-10))));
        purpleBackgroundTwoVisorsRandomRain.onTrue(new InstantCommand(() -> leds.setEffect(new SolidColorEffect(LED_LENGTH, new int[] {100, 255, 0}),
                                                                                            new VisorEffect(LED_LENGTH, Colors.WHITE, VISOR_SIZE, VISOR_SPEED),
                                                                                            new VisorEffect(LED_LENGTH, Colors.BLUE, VISOR_SIZE, VISOR_SPEED, LED_LENGTH-10),
                                                                                            new RainEffect(LED_LENGTH, RAIN_SPEED, RAIN_VOLUME))));
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