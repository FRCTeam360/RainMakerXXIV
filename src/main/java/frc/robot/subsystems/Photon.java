// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Objects;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Photon extends SubsystemBase {
  public static enum TargetType {
    SPEAKER,
    STAGE
  }

  /** Creates a new Photon. */
  private final String[] cameraNames = { "lime_14" };
  private ArrayList<PhotonCamera> cameras = new ArrayList<>();
  PhotonCamera mCamera = new PhotonCamera("lime_14");
  private ArrayList<PhotonPoseEstimator> poseEstimators = new ArrayList<>();

  // this Should be the poseestimator for the center of the robot relative to the
  // april tag
  private double lastEstTimestamp = 0;

  // Cam mounted facing forward, half a meter forward of center, half a meter up
  // from center. THESE ARE NOT RIGHT AS OF 2/11/24
  public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
      new Rotation3d(0, 0, 0));

  // The standard deviations of our vision estimated poses, which affect
  // correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  // TODO: FIX THESE NUMBERS FOR
  private final double CAMERA_HEIGHT_METERS = 0.25;
  private final double TARGET_HEIGHT_METERS = 0.25;
  private final double CAMERA_PITCH_RADIANS = 0.0;

  public Photon() {
    for (String cameraName : cameraNames) {
      cameras.add(new PhotonCamera(cameraName));
    }
    for (PhotonCamera camera : cameras) {
      poseEstimators
          .add(new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam));
    }
    for (PhotonPoseEstimator estimator : poseEstimators) {
      estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    // we'll have to play around with this
    setupShuffleboard();
  }

  private void setupShuffleboard() {
    ShuffleboardTab visionTab = Shuffleboard.getTab("Vision");
    
    visionTab.addDouble("Pitch or ty", () -> {
      if(!Objects.isNull(mCamera) && mCamera.getLatestResult().hasTargets()){
        return mCamera.getLatestResult().getBestTarget().getPitch();
      }
      return 0.0;
    });
    visionTab.addDouble("Yaw or tx", () -> {
      if(!Objects.isNull(mCamera) && mCamera.getLatestResult().hasTargets()){
        return mCamera.getLatestResult().getBestTarget().getYaw();
      }
      return 0.0;
    });
  }

  public ArrayList<PhotonPipelineResult> getPipelineResult() {
    ArrayList<PhotonPipelineResult> pipelineResults = new ArrayList<>();
    for (PhotonCamera camera : cameras) {
      pipelineResults.add(camera.getLatestResult());
    }
    return pipelineResults;
  }

  public PhotonTrackedTarget getTargetInView(TargetType targetType) {
    // select the correct target based on alliance and target type
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    // initialize target number to 0
    int targetNumber = 0;
    if (alliance == DriverStation.Alliance.Blue) {
      if (targetType == TargetType.SPEAKER) {
        // target is the speaker
        targetNumber = 7;
      } else {
        // target is the stage
        targetNumber = 14;
      }
    } else {
      if (targetType == TargetType.SPEAKER) {
        // target is the speaker
        targetNumber = 4;
      } else {
        // target is the stage
        targetNumber = 13;
      }
    }
    if(!Objects.isNull(mCamera)){
      for (PhotonTrackedTarget target : mCamera.getLatestResult().getTargets()) {
        if (target.getFiducialId() == targetNumber) {
          return target;
        }
      }
    }
    return null;
  }

  public boolean isTargetInView(TargetType targetType) {
    return !Objects.isNull(targetType);
  }

  public double getSpecifiedTargetYaw(TargetType targetType) {
    // select the correct target based on alliance and target type
    PhotonTrackedTarget target = getTargetInView(targetType);
    if(!Objects.isNull(target)){
      return target.getYaw();
    }
    return 0.0;
  }

  public ArrayList<Boolean> getHasTarget() {
    ArrayList<Boolean> hasTargets = new ArrayList<>();
    for (PhotonCamera camera : cameras) {
      hasTargets.add(camera.getLatestResult().hasTargets());
    }
    return hasTargets;
  }

  // https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/apriltagvision/AprilTagVision.java

  public ArrayList<Double> getYaws() {
    ArrayList<Double> angles = new ArrayList<>();
    for (PhotonCamera camera : cameras) {
      angles.add(camera.getLatestResult().getBestTarget().getYaw());
    }
    return angles;
  }

  public ArrayList<Double> getPitches() {
    ArrayList<Double> angles = new ArrayList<>();
    for (PhotonCamera camera : cameras) {
      angles.add(camera.getLatestResult().getBestTarget().getPitch());
    }
    return angles;
  }

  // TODO im not gonna change anything in here because this is lowkey weird since
  // you cant have vars in arrays plus visEst is var so its not even compatible
  // with the return type (possibly)
  // this is the projected pose estimation from the camera
  // public Optional<EstimatedRobotPose> getEstimatedGlobalPoses() {
  // // for(PhotonPoseEstimator estimator : poseEstimators) {

  // // }
  // var visionEst = photonEstimator13.update();
  // double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
  // boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
  // // This is missing alot from the example that might or might not be important
  // if (newResult) lastEstTimestamp = latestTimestamp;

  // return visionEst;
  // }

  // public double getTimeStamp() {
  // double timestamp = camera.getLatestResult().getTimestampSeconds();
  // return timestamp;
  // }

  // public Pose3d getPose() {
  // Pose3d pose = photonEstimator13.getReferencePose();
  // return pose;
  // }

  // TODO lmao this method is already so messed up im not even gonna try to change
  // this
  // // this is some logic for rejecting bad poses
  // public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
  // var estStdDevs = kSingleTagStdDevs;

  // var targets = getPipelineResult().getTargets();
  // // int numTags = 0;
  // // double avgDist = 0;
  // // for (var tgt : targets) {
  // // var tagPose =
  // photonEstimator13.getFieldTags().getTagPose(tgt.getFiducialId());
  // // if (tagPose.isEmpty()) continue;
  // // numTags++;
  // // avgDist +=
  // //
  // tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
  // // }
  // // if (numTags == 0) return estStdDevs;
  // // avgDist /= numTags;

  // // // Decrease std devs if multiple targets are visible
  // // if (numTags > 1) estStdDevs = kMultiTagStdDevs; // if larger than one make
  // multiple
  // // // Increase std devs based on (average) distance
  // // if (numTags == 1 && avgDist > 4) // reject all poses that are based on one
  // tag more than 4 feet away
  // // estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
  // Double.MAX_VALUE);
  // // else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30)); //
  // multiple table by d^2 / 30. arbitrary number

  // return estStdDevs;

  // }

  // public void getRange() {
  // var result = camera.getLatestResult();
  // if (result.hasTargets()) {
  // // First calculate range
  // double range =
  // PhotonUtils.calculateDistanceToTargetMeters(
  // CAMERA_HEIGHT_METERS,
  // TARGET_HEIGHT_METERS,
  // CAMERA_PITCH_RADIANS,
  // Units.degreesToRadians(result.getBestTarget().getPitch()));
  // }
  // }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Pitch/ty", mCamera.getLatestResult().getBestTarget().getPitch());
  }
}
