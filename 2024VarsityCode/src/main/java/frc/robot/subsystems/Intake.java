// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intake;
    private CANSparkMax helper;

    private double intakeSpeed = 0.0;

    /** Creates a new Intake. */
    public Intake() {
        intake = new CANSparkMax(Constants.Intake.primaryCAN, MotorType.kBrushless);
        helper = new CANSparkMax(Constants.Intake.helperCAN, MotorType.kBrushless);
    }

    public void setIntakeSpeed(double speed){
        intakeSpeed = -speed; // CHANGE reversed intake speed
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        intake.set(intakeSpeed);
        helper.set(intakeSpeed);

        SmartDashboard.putNumber("Intake Speed", intakeSpeed);
    }
}
