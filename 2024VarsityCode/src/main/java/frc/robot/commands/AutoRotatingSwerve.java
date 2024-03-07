package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

public class AutoRotatingSwerve extends Command{
    
    private double rotation;
    private Translation2d translation;
    private boolean openLoop;

    private Swerve s_Swerve;
    private Limelight s_LL;
    private CommandPS4Controller controller;

    private PIDController rotationController;

    /*
     * Driver Control command
     * @param s_Swerve Swerve subsystem
     * @param controller PS4 controller
     * @param openLoop True
     */
    public AutoRotatingSwerve(Swerve s_Swerve, Limelight s_LL, CommandPS4Controller controller, boolean openLoop){
        // TODO
        rotationController = new PIDController(0.004, 0.0, 0.000);

        this.s_Swerve = s_Swerve;
        this.s_LL = s_LL;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.openLoop = openLoop;
    }

    @Override
    public void execute(){
        double yAxis = -controller.getLeftY();
        double xAxis = -controller.getLeftX();

        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;

        /* Slow Trigger */
        double slowdown = 1 - ((controller.getR2Axis()+1.0)/2.0 < Constants.stickDeadband ? 0 : (controller.getR2Axis()+1.0)/2.0);
        yAxis *= slowdown;
        xAxis *= slowdown;

        /* Slowdown from Pivot */
        Pivot pivot = RobotContainer.s_Pivot;
        if(pivot.getAngle() > 65){
            yAxis *= 0.5;
            xAxis *= 0.5;
        }

        SmartDashboard.putNumber("autorotate/setpoint", 0.0);
        SmartDashboard.putNumber("autorotate/offset", s_LL.targetXOffset());

        /* Calculates inputs for swerve subsystem */
        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rotationController.calculate(s_LL.targetXOffset(), 0.0) * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, openLoop);
    }
}
