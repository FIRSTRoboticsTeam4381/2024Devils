// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonVision extends SubsystemBase {
    private PhotonCamera cam1;
    private PhotonCamera cam2;

    private PhotonPoseEstimator est1;
    private PhotonPoseEstimator est2;

    private final AprilTagFieldLayout apriltagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    // TODO enable multi-tag layout in photonvision UI

    /** Creates a new PhotonVision. */
    public PhotonVision() {
        cam1 = new PhotonCamera("camera1"); // TODO change to match name in PhotonVision UI
        Transform3d cam1location = new Transform3d(0,0,0, new Rotation3d(0,0,0)); //TODO

        cam2 = new PhotonCamera("camera2"); // TODO change to match name in PhotonVision UI
        Transform3d cam2location = new Transform3d(0,0,0, new Rotation3d(0,0,0)); //TODO

        est1 = new PhotonPoseEstimator(apriltagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam1, cam1location); //TODO could also use AVG if can't configure in UI
        est2 = new PhotonPoseEstimator(apriltagLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam2, cam2location);
    }

    @Override
    public void periodic() {
        // .addVisionMeasurement()
        Pose3d pose1;
        Pose3d pose2;

        if(est1.update().isPresent()) pose1 = est1.update().get().estimatedPose;
        else pose1 = new Pose3d();
        if(est2.update().isPresent()) pose2 = est2.update().get().estimatedPose;
        else pose2 = new Pose3d();

        RobotContainer.s_Swerve.addPoseToField("cam1Pose", pose1.toPose2d());
        RobotContainer.s_Swerve.addPoseToField("cam2Pose", pose2.toPose2d());

        // This method will be called once per scheduler run
    }
}
