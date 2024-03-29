// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

public class AutoPivot extends Command {
  //private Shooter shooter;
  private Pivot pivot;
  private Swerve swerve;
  private Limelight ll;

  private double currentAngle = 30.0; // TODO podium angle

  /** Creates a new AutoAim. */
  public AutoPivot(Pivot pivot, Limelight ll, Swerve swerve) {
    //this.shooter = shooter;
    this.pivot = pivot;
    this.ll = ll;
    this.swerve = swerve;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private void calcAngle(){
    double predictedPosition = predictFuturePosition();
    if(ll.distanceFromGoal() >= 0.3){ // Only take predicted position if it is a reasonable positive number. Prevents calculation of infinity.
      double calculatedAngle = 56.12307316*Math.pow(predictedPosition, -0.5249909159627)-2; // r^2 = 0.995 // despite predicting, still won't move unless a target is in sight. prevents going crazy
      if(calculatedAngle <= 75){ // Only set current angle to the calculated angle if it was calculated to be less than 60
        currentAngle = calculatedAngle;
      }
    }
  }

  private double getTargetRelativeVelocity(){
    double robotVelocity = swerve.getRobotRelativeSpeeds().vxMetersPerSecond; // Since the Limelight is on the front of the robot, the only helpful velocity is the axis that is facing the target
    return -robotVelocity;
  }

  private double estimateDistance(){
    double lastDistance = ll.distanceFromGoal();
    double predictedTravelThroughLatency = getTargetRelativeVelocity() * (ll.totalLatency()/1000.0);
    double estimatedPostLatencyPosition = lastDistance - predictedTravelThroughLatency;
    return estimatedPostLatencyPosition;
  }

  private double predictFuturePosition(){
    final double lengthOfTime = 400; // ms
    double predictedTravel = getTargetRelativeVelocity() * (lengthOfTime/1000.0);
    double predictedPosition = estimateDistance() - predictedTravel;
    return predictedPosition;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calcAngle();
    //calcVelocity();

    SmartDashboard.putNumber("autoaim/Calculated Angle", currentAngle);
    SmartDashboard.putNumber("autoaim/Target Relative Velocity", getTargetRelativeVelocity());
    SmartDashboard.putNumber("autoaim/Estimated Latency Position", estimateDistance());
    SmartDashboard.putNumber("autoaim/Predicted Future Position", predictFuturePosition());

    pivot.setAngleReference(currentAngle, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //CommandScheduler.getInstance().schedule(pivot.goToTransit().alongWith(shooter.instantStopAll()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
