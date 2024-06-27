// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.ejml.simple.SimpleMatrix;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private PhotonCamera camC;
  private PhotonCamera camD;
  private Transform3d robotToCamC;
  private Transform3d robotToCamD;
  private PhotonPoseEstimator poseEstimateC;
  private PhotonPoseEstimator poseEstimateD;
  private Pose3d pose;

  private Matrix<N3,N1> camCMatrix = new Matrix<N3,N1>(SimpleMatrix.filled(3,1,100));
  private Matrix<N3,N1> camDMatrix = new Matrix<N3,N1>(SimpleMatrix.filled(3,1,100));

  /*
   * Testing Notes:
   * Successfully got it to work with different numbers. 0 trusted it perfectly,
   * 1 trusted it a bit slower, 10000 didn't move and 1000 hardly moved, 100 was v slow
   */

  StructPublisher<Pose3d> publisherC = NetworkTableInstance.getDefault()
    .getStructTopic("Camera_C (1)", Pose3d.struct).publish();
    
  StructPublisher<Pose3d> publisherD = NetworkTableInstance.getDefault()
    .getStructTopic("Camera_D", Pose3d.struct).publish();
    

  public Camera() {
    camC = new PhotonCamera("Camera_C (1)");
    camD = new PhotonCamera("Camera_D");
    robotToCamC = new Transform3d(new Translation3d(-0.3, 0.26, 0.21), new Rotation3d(0,-45/180.0*Math.PI,225.0/180.0*Math.PI));
    robotToCamD = new Transform3d(new Translation3d(-0.3, -0.26, 0.21), new Rotation3d(0,-45/180.0*Math.PI,135.0/180.0*Math.PI));
    poseEstimateC = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camC, robotToCamC);
    poseEstimateD = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camD, robotToCamD);
  }


  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> c = poseEstimateC.update();
    Optional<EstimatedRobotPose> d = poseEstimateD.update();

    List<PhotonTrackedTarget> camCTargets = camC.getLatestResult().getTargets();
    double area = 0;
    for(PhotonTrackedTarget t : camCTargets){
      area+=t.getArea();
    }
    SmartDashboard.putNumber("Cam C Target Area", area);
    camCMatrix.fill(area<0.5?75:0);

    List<PhotonTrackedTarget> camDTargets = camD.getLatestResult().getTargets();
    area=0;
    for(PhotonTrackedTarget t : camDTargets){
      area+=t.getArea();
    }
    SmartDashboard.putNumber("Cam D Target Area", area);
    camDMatrix.fill(area<0.5?75:0);

    /*
     * Testing Notes:
     * Got it to work with a hard cutoff, it's super cool. I think the area scale
     * is between 0-1.5 with this year's field, you don't really get more than 1.5.
     * I think next we're going to linearly adjust trust based on area.
     */

    //camCMatrix.
    if(c.isPresent()) {
      EstimatedRobotPose pose = c.get();
      publisherC.set(pose.estimatedPose);
      RobotContainer.s_Swerve.mSwerveOdometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, camCMatrix);
    }else{
      pose = new Pose3d();
    }

    if(d.isPresent()) {
      EstimatedRobotPose pose = d.get();
      publisherD.set(pose.estimatedPose);
      RobotContainer.s_Swerve.mSwerveOdometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, camDMatrix);
    }else{
      pose = new Pose3d();
    }
  }

  public boolean GetTargets() {
    var result = camC.getLatestResult();
    boolean hasTargets = result.hasTargets();
    return hasTargets;
  }


  public double GetTargetYaw() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double yaw = target.getYaw();
    return yaw;
  }

  public double GetTargetPitch() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double pitch = target.getPitch();
    return pitch;
  }

  public double GetTargetArea() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double area = target.getArea();
    return area;
  }

  public int GetAprilTagID() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    int targetID = target.getFiducialId();
    return targetID;
  }

  public double GetAprilTagPoseAmbiguity() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double poseAmbiguity = target.getPoseAmbiguity();
    return poseAmbiguity;
  }

  public void GetAprilTagPose() {
    var result = camC.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    // Oddly doesn't work
    //Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
  }
}
