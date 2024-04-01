// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class AutoShooter extends Command {
  //private Shooter shooter;
  private Pivot pivot;
  private Swerve swerve;
  private Limelight ll;
  private Shooter shooter;

  private double currentAngle = 30.0; // TODO podium angle
  private double currentVelocity = 0.0;

  /** Creates a new AutoAim. */
  public AutoShooter(Pivot pivot, Shooter shooter, Limelight ll, Swerve swerve) {
    //this.shooter = shooter;
    this.pivot = pivot;
    this.ll = ll;
    this.swerve = swerve;
    this.shooter = shooter;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private void calcAngle(){
    double llDistance = ll.distanceFromGoal();
    double predictedPosition = ll.predictFuturePosition();
    if(llDistance >= 0.3 && llDistance <= 9){ // Only take predicted position if it is a reasonable positive number. Prevents calculation of infinity.
      double calculatedAngle = 51.16053558 * Math.pow(predictedPosition, -0.5133988343)+1; // r^2 = 0.995 // despite predicting, still won't move unless a target is in sight. prevents going crazy
      if(calculatedAngle <= 75){ // Only set current angle to the calculated angle if it was calculated to be less than 60
        currentAngle = calculatedAngle;
      }
    }
  }
  private void calcVelocity(){
    double llDistance = ll.distanceFromGoal();
    double predictedPosition = ll.predictFuturePosition();
    if(llDistance >= 0.3 && llDistance <= 9){
      double calculatedVelocity = 3898.436881 * Math.pow(1.054948756, predictedPosition)+250;
      if(calculatedVelocity <= 6300 && calculatedVelocity >= 4000){
        currentVelocity = calculatedVelocity;
      }
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    calcAngle();
    calcVelocity();

    SmartDashboard.putNumber("autoaim/Calculated Angle", currentAngle);
    SmartDashboard.putNumber("autoaim/Calculated Velocity", currentVelocity);

    pivot.setAngleReference(currentAngle, 1);
    shooter.setVelocity(currentVelocity, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //CommandScheduler.getInstance().schedule(pivot.goToTransit().alongWith(shooter.instantStopAll()));
    shooter.setPercOutput(0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
