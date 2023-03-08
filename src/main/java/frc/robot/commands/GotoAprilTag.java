// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.stream.Stream;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import static frc.robot.Constants.Constraints.*;
import static frc.robot.Constants.VisionConfigs.*;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Drivebase;

import org.photonvision.targeting.PhotonTrackedTarget;

public class GotoAprilTag extends CommandBase {
  /** PID controllers */
  private ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, X_CONSTRAINTS);
  private ProfiledPIDController yController = new ProfiledPIDController(0, 0, 0, Y_CONSTRAINTS);
  private ProfiledPIDController rController = new ProfiledPIDController(0, 0, 0, THETA_CONSTRAINTS);

  /** Robot pose */
  private Pose2d robotPose2d;

  /**
   * Apriltag ID to aim
   * If apriltag ID is set to -1, then the dest target is set to best target
   */
  private int destTag;

  private Vision m_Vision;
  private Drivebase m_drivebase;

  /** Creates a new GotoAprilTag. */
  public GotoAprilTag(int destTag, Drivebase __drivebase, Vision __vision) 
  {
    m_Vision = __vision;
    m_drivebase = __drivebase;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Vision);
    addRequirements(m_drivebase);

    this.destTag = destTag;
    robotPose2d = m_Vision.getPose2d();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Config PID controllers
    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    rController.setTolerance(0.2);
    rController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Update the 2D pose value
    robotPose2d = m_Vision.getPose2d();

    // Reset PID controllers to the current robotPose values
    xController.reset(robotPose2d.getX());
    yController.reset(robotPose2d.getY());
    rController.reset(robotPose2d.getRotation().getRadians());

    // Create 3D robot pose based on the calculated 2D pose
    Pose3d robotPose3d = new Pose3d(
      robotPose2d.getX(),
      robotPose2d.getY(),
      0,
      new Rotation3d(0, 0, robotPose2d.getRotation().getRadians())
    );

    // Get apriltag object from the pipeline
    var photonRes = limelight.getLatestResult();
    if (photonRes.hasTargets()) {
      // Got apriltag
      PhotonTrackedTarget target;

      if (destTag == -1) {
        // Get best target
        target = photonRes.getBestTarget();
      } else {
        // Get a specific target
        Stream<PhotonTrackedTarget> targetStream = photonRes.getTargets().stream()
        .filter(t -> t.getFiducialId() == destTag)  // Filter target with destTag id
        .filter(t -> t.getPoseAmbiguity() <= 0.2);  // Filter target with ambiguity < 0.2

        if (targetStream.count() <= 0) {
          // None target available
          m_drivebase.drive(0, 0, 0); // Stop driving
          return;
        }

        target = targetStream.findFirst().get();
      }

      // Calculate different poses
      /*
       * var cameraPose = robotPose3d.transformBy(robotToLimelight);
       * var camToTarget = target.getBestCameraToTarget();
       * var targetPose = cameraPose.transformBy(camToTarget);
       * var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
       * 
       * These lines can be turned into one line like below:
       */

      Pose2d goalPose = robotPose3d.transformBy(ROBOT_TO_LIMELIGHT).transformBy(target.getBestCameraToTarget()).transformBy(TAG_TO_GOAL).toPose2d();

      // Set goal for the PID controllers
      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
      rController.setGoal(goalPose.getRotation().getRadians());

      // Calculate the speed for each poses and stop when reached the goal

      var xSpeed = xController.calculate(robotPose3d.getX());
      if(xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed = yController.calculate(robotPose3d.getY());
      if(yController.atGoal()) {
        ySpeed = 0;
      }

      var rSpeed = rController.calculate(robotPose2d.getRotation().getRadians());
      if(rController.atGoal()) {
        rSpeed = 0;
      }

      m_drivebase.drive(xSpeed, ySpeed, rSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivebase.drive(0, 0, 0); // Should stop immidiately
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
