// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.lib.util.SparkOptimizer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    private CANSparkMax helper;

    public static final double INTAKE_SPEED = 0.5;

    /** Creates a new Intake. */
    public Intake() {
        intake = new CANSparkMax(Constants.Intake.primaryCAN, MotorType.kBrushless);
        helper = new CANSparkMax(Constants.Intake.helperCAN, MotorType.kBrushless);

        helper.follow(intake);

        SparkOptimizer.optimizeFrames(intake, true, false, false, false, false, false);
        SparkOptimizer.optimizeFrames(helper, false, false, false, false, false, false);
    }

    public void setIntakeSpeed(double speed){
        intake.set(-speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        SmartDashboard.putNumber("Intake Speed", intake.get());
    }
}
