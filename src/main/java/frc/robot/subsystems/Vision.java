// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.ASUtil;
import frc.robot.Constants;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Vision extends SubsystemBase {
  private Swerve swerve;
  private PhotonCamera cam;
  private PhotonCamera camBack;

  private PhotonPoseEstimator estimator;

  public double[] camPose = new double[7];

  PIDController rotationController = new PIDController(0.1, 0, 0);


  /** Creates a new Vision. */
  public Vision() {
    cam = new PhotonCamera(Constants.Vision.CAM_NAME);
    camBack = new PhotonCamera(Constants.Vision.CAM_NAME_BACK);
    cam.setDriverMode(false);
    camBack.setDriverMode(false);

    estimator = 
      new PhotonPoseEstimator(
        Constants.FieldConstants.APRIL_TAG_FIELD_LAYOUT,
        PoseStrategy.MULTI_TAG_PNP_ON_RIO,
        cam,
        Constants.Vision.ROBOT_TO_CAM_TRANSFORM);

    estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  public List<Optional<EstimatedRobotPose>> getEstimatedPoses(){
    Optional<EstimatedRobotPose> pose = estimator.update();

    pose.ifPresent((x) -> camPose = ASUtil.pose3dToDoubleArray(x.estimatedPose));

    return List.of(pose);
  }

  public double calculateOffset(){
    var result = cam.getLatestResult();
    double roatationSpeed;

    if (result.hasTargets()) {
      double yaw = (result.getBestTarget().getYaw());
      roatationSpeed = -rotationController.calculate(yaw, 0);
    } 
    else{roatationSpeed = 0;}
    return roatationSpeed;
  }

  public double calculateOffset(int targetID) {
    var result = cam.getLatestResult();
    double roatationSpeed;

    if (result.hasTargets()) {
      double yaw = (result.getTargets().get(targetID).getYaw());
      roatationSpeed = -rotationController.calculate(yaw, 0);
    } 
    else{roatationSpeed = 0;}
    return roatationSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (var pose : getEstimatedPoses()) {
        if (pose.isPresent()) {
          swerve.resetOdometry(pose.get().estimatedPose.toPose2d());
        }
      }
        


    List<Pose2d> visionPoses = 
      getEstimatedPoses().stream()
        .filter(Optional::isPresent)
        .map((p) -> p.get().estimatedPose.toPose2d())
        .toList();
    Swerve.odometry.getObject("visionPoses").setPoses(visionPoses);
  }
}
