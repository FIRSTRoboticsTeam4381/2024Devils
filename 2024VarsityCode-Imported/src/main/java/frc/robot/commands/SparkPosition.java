// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SparkPosition extends Command{
    /** Creates a new SparkMaxPosition */
    //private CANSparkBase motor;
    private SparkClosedLoopController controller;
    private double position;
    private ClosedLoopSlot slotNumber;
    private double error;
    private Supplier<Double> feedback;

    public SparkPosition(SparkClosedLoopController c, double pos, ClosedLoopSlot slot, double err, Subsystem s, Supplier<Double> feedback){
        controller=c;
        position=pos;
        slotNumber=slot;
        error=err;
        this.feedback=feedback;
        addRequirements(s);
    }
    public SparkPosition(SparkBase m, double pos, ClosedLoopSlot slot, double err, Subsystem s, Supplier<Double> feedback){
        this(m.getClosedLoopController(), pos, slot, err, s, feedback);
        // Use addRequirements() here to declare system dependencies
    }
    public SparkPosition(SparkBase m, double pos, ClosedLoopSlot slot, double err, Subsystem s){
        this(m, pos, slot, err, s, getFeedbackSource(m));
    }
    public SparkPosition(SparkBase m, double pos, double err, Subsystem s){
        this(m, pos, ClosedLoopSlot.kSlot0, err, s, getFeedbackSource(m));
    }


    private static Supplier<Double> getFeedbackSource(SparkBase m)
    {
        if(m.getClass() == SparkMax.class)
        {
            switch(((SparkMax) m).configAccessor.closedLoop.getFeedbackSensor())
            {
                case kAbsoluteEncoder:
                    return m.getAbsoluteEncoder()::getPosition;
                case kAlternateOrExternalEncoder:
                    return ((SparkMax) m).getAlternateEncoder()::getPosition;
                case kAnalogSensor:
                    return ((SparkMax) m).getAnalog()::getPosition;
                case kNoSensor:
                    return null;
                case kPrimaryEncoder:
                    return ((SparkMax) m).getEncoder()::getPosition;
                default:
                    return null;
            }
        }
        else if(m.getClass() == SparkFlex.class)
        {
            switch(((SparkFlex) m).configAccessor.closedLoop.getFeedbackSensor())
            {
                case kAbsoluteEncoder:
                    return m.getAbsoluteEncoder()::getPosition;
                case kAlternateOrExternalEncoder:
                    return ((SparkFlex) m).getExternalEncoder()::getPosition;
                case kAnalogSensor:
                    return ((SparkFlex) m).getAnalog()::getPosition;
                case kNoSensor:
                    return null;
                case kPrimaryEncoder:
                    return ((SparkFlex) m).getEncoder()::getPosition;
                default:
                    return null;
            }
        }
        else
        {
            // idk what motor controller this is
            return null;
        }
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
