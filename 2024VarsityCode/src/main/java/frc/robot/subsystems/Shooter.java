// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private CANSparkMax topShooter;
    private CANSparkMax bottomShooter;
    private CANSparkMax topDirectional;
    private CANSparkMax bottomDirectional;

    private double tsSpeed = 0;
    private double bsSpeed = 0;
    private double tdSpeed = 0;
    private double bdSpeed = 0;

    /** Creates a new Shooter. */
    public Shooter() {
        topShooter = new CANSparkMax(Constants.Shooter.topShooterCAN, MotorType.kBrushless);
        bottomShooter = new CANSparkMax(Constants.Shooter.bottomShooterCAN, MotorType.kBrushless);
        topDirectional = new CANSparkMax(Constants.Shooter.topDirectionalCAN, MotorType.kBrushless);
        bottomDirectional = new CANSparkMax(Constants.Shooter.bottomDirectionalCAN, MotorType.kBrushless);
    }

    public void setShooterSpeeds(double speed){
        setTopShooterSpeed(speed); // TODO reverse?
        setBottomShooterSpeed(speed); // TODO reverse?
    }
    public void setTopShooterSpeed(double speed){
        tsSpeed = speed;
    }
    public void setBottomShooterSpeed(double speed){
        bsSpeed = speed;
    }
    public void setTopDirectionalSpeed(double speed){
        tdSpeed = speed;
    }
    public void setBottomDirectionalSpeed(double speed){
        bdSpeed = speed;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        topShooter.set(tsSpeed);
        bottomShooter.set(bsSpeed);
        topDirectional.set(tdSpeed);
        bottomDirectional.set(bdSpeed);
    }
}
