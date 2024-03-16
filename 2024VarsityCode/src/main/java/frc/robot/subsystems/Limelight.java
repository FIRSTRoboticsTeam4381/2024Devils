package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase{
    private double currentAngle = 30;

    private NetworkTable getLimelight(){
        return NetworkTableInstance.getDefault().getTable("limelight-devils");
    }


    /* CONSTRUCTORS */

    public Limelight(){
        getLimelight().getEntry("stream").setNumber(0);
        setDrivingMode(false);
    }


    /* 2D TARGETING */

    public double targetXOffset(){
        return (double) getLimelight().getEntry("tx").getNumber(0);
    }
    public double targetYOffset(){
        return (double) getLimelight().getEntry("ty").getNumber(0);
    }
    public double targetArea(){
        return (double) getLimelight().getEntry("ta").getNumber(0);
    }
    public double pipelineLatency(){
        return (double) getLimelight().getEntry("tl").getNumber(0);
    }
    public double captureLatency(){
        return (double) getLimelight().getEntry("cl").getNumber(0);
    }
    public double totalLatency(){
        return captureLatency()+pipelineLatency();
    }

    private void calcAngle(){
        double predictedPosition = predictFuturePosition();
        if(distanceFromGoal() >= 0.3){ // Only take predicted position if it is a reasonable positive number. Prevents calculation of infinity.
          double calculatedAngle = 57.62307316*Math.pow(predictedPosition, -0.5549909159627); // r^2 = 0.995 // despite predicting, still won't move unless a target is in sight. prevents going crazy
          if(calculatedAngle <= 75){ // Only set current angle to the calculated angle if it was calculated to be less than 60
            currentAngle = calculatedAngle;
          }
        }
      }
    
      private double getTargetRelativeVelocity(){
        double robotVelocity = RobotContainer.s_Swerve.getRobotRelativeSpeeds().vxMetersPerSecond; // Since the Limelight is on the front of the robot, the only helpful velocity is the axis that is facing the target
        return -robotVelocity;
      }
    
      private double estimateDistance(){
        double lastDistance = distanceFromGoal();
        double predictedTravelThroughLatency = getTargetRelativeVelocity() * (totalLatency()/1000.0);
        double estimatedPostLatencyPosition = lastDistance - predictedTravelThroughLatency;
        return estimatedPostLatencyPosition;
      }
    
      private double predictFuturePosition(){
        final double lengthOfTime = 400; // ms
        double predictedTravel = getTargetRelativeVelocity() * (lengthOfTime/1000.0);
        double predictedPosition = estimateDistance() - predictedTravel;
        return predictedPosition;
      }


    /* ROBOT POSE ESTIMATION */

    public double distanceFromGoal(){
        double[] targetPose = {0,0,0};
        double[] relativeRobotPose = cameraPoseFromTarget();

        double distance = Math.sqrt(Math.pow(targetPose[0]-relativeRobotPose[0], 2) + Math.pow(targetPose[2]-relativeRobotPose[2], 2));
        return distance;
    }
    public double[] botPoseFromField(){
        double[] position = getLimelight().getEntry("botpose").getDoubleArray(new double[] {0,0,0});
        return position;
    }
    public double[] botPoseFromBlue(){
        double[] position = getLimelight().getEntry("botpose_wpiblue").getDoubleArray(new double[] {0,0,0});
        return position;
    }
    public double[] botPoseFromRed(){
        double[] position = getLimelight().getEntry("botpose_wpired").getDoubleArray(new double[] {0,0,0});
        return position;
    }
    public double[] cameraPoseFromTarget(){
        double[] position = getLimelight().getEntry("camerapose_targetspace").getDoubleArray(new double[] {0,0,0});
        return position;
    }
    public double[] targetPoseFromCamera(){
        double[] position = getLimelight().getEntry("targetpose_cameraspace").getDoubleArray(new double[] {0,0,0});
        return position;
    }
    public double[] targetPoseFromRobot(){
        double[] position = getLimelight().getEntry("targetpose_robotspace").getDoubleArray(new double[] {0,0,0});
        return position;
    }
    public double[] botPoseFromTarget(){
        double[] position = getLimelight().getEntry("botpose_targetspace").getDoubleArray(new double[] {0,0,0});
        return position;
    }
    public double[] cameraPoseFromRobot(){
        double[] position = getLimelight().getEntry("camerapose_robotspace").getDoubleArray(new double[] {0,0,0});
        return position;
    }


    /* STATUS */

    public int hasTargets(){
        int targets = (int) getLimelight().getEntry("tv").getInteger(0);
        return targets;
    }
    public int primaryTargetID(){
        return (int) getLimelight().getEntry("tid").getInteger(0);
    }


    /* CAMERA CONTROLS */

    private void setDrivingMode(boolean b){
        setCamMode(b?1:0);
    }
    private void setLEDMode(int mode){
        if(validateInput(mode, 1, 3)){
            getLimelight().getEntry("ledMode").setNumber(mode);
        }
    }
    private void setCamMode(int mode){
        if(validateInput(mode, 0, 1)){
            getLimelight().getEntry("camMode").setNumber(mode);
        }
    }
    private void setPipeline(int pl){
        if(validateInput(pl, 0, 9)){
            getLimelight().getEntry("pipeline").setNumber(pl);
        }
    }
    private void setStreamMode(int mode){
        if(validateInput(mode, 0, 2)){
            getLimelight().getEntry("stream").setNumber(mode);
        }
    }
    public void takeSnapshot(){
        getLimelight().getEntry("snapshot").setNumber(1);
    }
    public void resetSnapshot(){
        getLimelight().getEntry("snapshot").setNumber(0);
    }


    /* PERIODIC */

    @Override
    public void periodic(){
        SmartDashboard.putNumberArray("limelight/targetPose", cameraPoseFromTarget());
        SmartDashboard.putBoolean("limelight/targetInSight", hasTargets()==1.0);
        SmartDashboard.putNumber("limelight/goalDistance (Meters)", distanceFromGoal());
        SmartDashboard.putNumber("limelight/goalDistance (Feet)", distanceFromGoal()*3.281);
        SmartDashboard.putNumber("limelight/targetXOffset", targetXOffset());
        calcAngle();
        SmartDashboard.putNumber("limelight/Calculated Angle", currentAngle);
    }

    private boolean validateInput(int in, int... validRange){
        if(validRange.length == 2){
            for(int i = validRange[0]; i <= validRange[1]; i++){
                if(in == i) return true;
            }
        }else{
            for(int i = 0; i < validRange.length; i++){
                if(in == validRange[i]) return true;
            }
        }
        return false;
    }
}
