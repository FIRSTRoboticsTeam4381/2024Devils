package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    
      private double getTargetRelativeVelocity(){
        double robotVelocity = RobotContainer.s_Swerve.getRobotRelativeSpeeds().vxMetersPerSecond; // Since the Limelight is on the front of the robot, the only helpful velocity is the axis that is facing the target
        return -robotVelocity;
      }

      private double estimateError(){
        double distance = distanceFromGoal();
        double estimatedError = 0.1294509025 * Math.pow(1.235817166, distance);
        return estimatedError;
      }

      private double estimateErrorRed(){
        double distance = distanceFromGoal();
        double estimatedError = 0.0459039304 * Math.pow(distance, 1.345147159);
        return estimatedError;
      }
      private double estimateErrorBlue(){
        double distance = distanceFromGoal();
        double estimatedError = 0.0460594022 * Math.pow(distance, 1.224620869);
        return estimatedError;
      }

      private double estimateDistance(){
        Optional<Alliance> allianceOptional = DriverStation.getAlliance();
        Alliance alliance;
        if(allianceOptional.isPresent()) alliance=allianceOptional.get();
        else alliance=Alliance.Red;

        double lastDistance = distanceFromGoal();
        double error = alliance==Alliance.Blue ? estimateErrorBlue() : estimateErrorRed();
        double latency = totalLatency();
        double velocity = getTargetRelativeVelocity();
        double estimatedRealDistance = lastDistance+error;
        double predictedTravelThroughLatency = velocity * (latency/1000.0);
        double estimatedPosition = estimatedRealDistance - predictedTravelThroughLatency;
        return estimatedPosition;
      }
      public double predictFuturePosition(){
        final double lengthOfTime = 500;
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
        SmartDashboard.putNumber("limelight/Estimate Error", estimateError());
        SmartDashboard.putNumber("limelight/Estimated Distance", estimateDistance());
        SmartDashboard.putNumber("limelight/Predicted Position", predictFuturePosition());
        //SmartDashboard.putNumber("limelight/Calculated Angle", currentAngle);
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
