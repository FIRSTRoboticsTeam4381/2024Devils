// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Camera extends SubsystemBase {
  /** Creates a new Camera. */
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private PhotonCamera cam;
  private Transform3d robotToCam;
  private PhotonPoseEstimator poseEstimate;
  private Pose3d pose;

  StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Camera_C (1)", Pose3d.struct).publish();

  public Camera() {
    cam = new PhotonCamera("Camera_C (1)");
    robotToCam = new Transform3d(new Translation3d(0.3, -0.26, 0.21), new Rotation3d(0,-45/180.0*Math.PI,45.0/180.0*Math.PI));
    poseEstimate = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
  }


  @Override
  public void periodic() {
    Optional<EstimatedRobotPose> x = poseEstimate.update();
    if(x.isPresent()) {
      pose = x.get().estimatedPose;
      publisher.set(pose);
    }else{
      pose = new Pose3d();
    }
    
  }

  public boolean GetTargets() {
    var result = cam.getLatestResult();
    boolean hasTargets = result.hasTargets();
    return hasTargets;
  }


  public double GetTargetYaw() {
    var result = cam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double yaw = target.getYaw();
    return yaw;
  }

  public double GetTargetPitch() {
    var result = cam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double pitch = target.getPitch();
    return pitch;
  }

  public double GetTargetArea() {
    var result = cam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double area = target.getArea();
    return area;
  }

  public int GetAprilTagID() {
    var result = cam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    int targetID = target.getFiducialId();
    return targetID;
  }

  public double GetAprilTagPoseAmbiguity() {
    var result = cam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    double poseAmbiguity = target.getPoseAmbiguity();
    return poseAmbiguity;
  }

  public void GetAprilTagPose() {
    var result = cam.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    // Oddly doesn't work
    //Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()), cameraToRobot);
  }
}
