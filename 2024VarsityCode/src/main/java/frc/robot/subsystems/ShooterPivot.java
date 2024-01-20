// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPivot extends SubsystemBase {
    /* Because of the 90 degree gearbox, these pivot motors should be able to run in the same direction */
    private CANSparkMax leftPivot;
    private CANSparkMax rightPivot;
    private RelativeEncoder leftPivotEncoder;
    private RelativeEncoder rightPivotEncoder;
    private AbsoluteEncoder pivotAbsoluteEncoder;
    private SparkPIDController pivotPID;

    private double setPoint = 0;
    private double pos = 0.0; // TODO
    
    /** Creates a new ShooterPivot. */
    public ShooterPivot() {
        leftPivot = new CANSparkMax(Constants.Shooter.leftPivotCAN, MotorType.kBrushless);
        rightPivot = new CANSparkMax(Constants.Shooter.rightPivotCAN, MotorType.kBrushless);

        leftPivotEncoder = leftPivot.getEncoder();
        rightPivotEncoder = rightPivot.getEncoder();
        pivotAbsoluteEncoder = rightPivot.getAbsoluteEncoder(Type.kDutyCycle);

        leftPivot.setIdleMode(IdleMode.kBrake);
        rightPivot.setIdleMode(IdleMode.kBrake);
        rightPivot.set(0);
        leftPivot.set(0);

        rightPivot.follow(leftPivot);

        pivotPID = leftPivot.getPIDController();
        pivotPID.setFeedbackDevice(pivotAbsoluteEncoder);
        pivotPID.setP(0);
        pivotPID.setI(0);
        pivotPID.setD(0);
        pivotPID.setFF(0);
        pivotPID.setOutputRange(-1, 1);
    }

    public void setPivotPosition(double encPos){
        setPoint = encPos;
    }
    public void setPivotAngle(double angle){
        //TODO convert from degrees to encoder ticks
    }

    public double getArmAbsolute(){
        return pivotAbsoluteEncoder.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        pivotPID.setReference(setPoint, ControlType.kPosition);
    }
}
