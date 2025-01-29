// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SparkPosition extends Command{
    /** Creates a new SparkMaxPosition */
    //private CANSparkBase motor;
    private SparkPIDController controller;
    private double position;
    private int slotNumber;
    private double error;
    private Supplier<Double> feedback;

    public SparkPosition(SparkPIDController c, double pos, int slot, double err, Subsystem s, Supplier<Double> feedback){
        controller=c;
        position=pos;
        slotNumber=slot;
        error=err;
        this.feedback=feedback;
        addRequirements(s);
    }
    public SparkPosition(CANSparkBase m, double pos, int slot, double err, Subsystem s, Supplier<Double> feedback){
        this(m.getPIDController(), pos, slot, err, s, feedback);
        // Use addRequirements() here to declare system dependencies
    }
    public SparkPosition(CANSparkBase m, double pos, int slot, double err, Subsystem s){
        this(m, pos, slot, err, s, m.getEncoder()::getPosition);
    }

    // Called when the command is initially scheduled
    @Override
    public void initialize(){
        controller.setReference(position, ControlType.kPosition, slotNumber);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted){}

    // Returns true when the command should end.
    @Override
    public boolean isFinished(){
        return Math.abs(position - feedback.get()) < error;
    }
}
