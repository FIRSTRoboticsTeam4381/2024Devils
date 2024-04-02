package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

public class TeleopSwerve extends Command{
    
    private double rotation;
    private Translation2d translation;
    private boolean openLoop;

    private Swerve s_Swerve;
    private CommandPS4Controller controller;

    /*
     * Driver Control command
     * @param s_Swerve Swerve subsystem
     * @param controller PS4 controller
     * @param openLoop True
     */
    public TeleopSwerve(Swerve s_Swerve, CommandPS4Controller controller, boolean openLoop){
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.openLoop = openLoop;
    }

    @Override
    public void execute(){
        double yAxis = -controller.getLeftY();
        double xAxis = -controller.getLeftX();
        double rAxis = -controller.getRightX();

        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        /* Slow Trigger */
        double slowdown = 1 - ((controller.getR2Axis()+1.0)/2.0 < Constants.stickDeadband ? 0 : (controller.getR2Axis()+1.0)/2.0);
        yAxis *= slowdown;
        xAxis *= slowdown;
        rAxis *= slowdown;

        /* Slowdown from Pivot */
        if(RobotContainer.s_Pivot.getAngle() > 65 && RobotContainer.s_Pivot.getAngle() < 350){
            yAxis *= 0.5;
            xAxis *= 0.5;
            rAxis *= 0.5;
        }

        /* Calculates inputs for swerve subsystem */
        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        SmartDashboard.putNumber("teleopSwerve/Controller yVel", translation.getX());
        SmartDashboard.putNumber("teleopSwerve/Controller xVel", translation.getY());
        fieldSpeeds();
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, controller.L1().getAsBoolean()?false:true, openLoop);
    }

    private void fieldSpeeds(){
        Rotation2d rotation = s_Swerve.getYaw();
        double robotY = s_Swerve.getRobotRelativeSpeeds().vxMetersPerSecond;
        double robotX = s_Swerve.getRobotRelativeSpeeds().vyMetersPerSecond;

        double fieldY = robotY*rotation.getCos() + robotX*rotation.getSin();
        double fieldX = robotY*rotation.getSin() + robotX*rotation.getCos();
        SmartDashboard.putNumber("teleopSwerve/Calculated yVel", fieldY);
        SmartDashboard.putNumber("teleopSwerve/Calculated xVel", fieldX);
    }
}
