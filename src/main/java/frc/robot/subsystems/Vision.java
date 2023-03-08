// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.Pair;

import static frc.robot.Constants.VisionConfigs.*;

public class Vision extends SubsystemBase {
  /** Creates a new PoseEstimator. */

  private AprilTagFieldLayout fieldLayout;
  private RobotPoseEstimator poseEstimator;

  public Vision() {
    // Load the field layout from the file
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(fieldLayoutFile);
    }

    catch (IOException e) {
      System.out.println("Error loading field layout file: " + e.getMessage());
    }

    // Create an array of camera objects and their 3D mount vectors.
    ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(limelight, ROBOT_TO_LIMELIGHT)); // First camera: limelight
    //camList.add(new Pair<PhotonCamera, Transform3d>(backcam, ROBOT_TO_BACKCAM)); // Second camera: camera faced backward
    
    // Create robotPoseEstimator object
    poseEstimator = new RobotPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);
  }

  public Pose2d getPose2d() {
    // Update the pose estimator
    var r = poseEstimator.update();
    
    // Get the new 3D pose, transform it into 2D X-Y plane pose and return
    return (r.isPresent() ? r.get().getFirst().toPose2d() : null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
