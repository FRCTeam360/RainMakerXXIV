// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Photon extends SubsystemBase {
  /** Creates a new Photon. */
  public final String kCameraName = "Inno_Cam2"  ;
  private final PhotonCamera camera;

// this Should be the poseestimator for the center of the robot relative to the april tag
// private final PhotonPoseEstimator photonEstimator13; 

private double lastEstTimestamp = 0;

        // Cam mounted facing forward, half a meter forward of center, half a meter up from center. THESE ARE NOT RIGHT AS OF 2/11/24
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
     
        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  //TODO: FIX THESE NUMBERS FOR 
  private final double CAMERA_HEIGHT_METERS = 0.25  ;
  private final double TARGET_HEIGHT_METERS = 0.25 ;
  private final double CAMERA_PITCH_RADIANS = 0.0;

  
  public Photon() {
    camera = new PhotonCamera(kCameraName);

      // photonEstimator13 =
    //    new PhotonPoseEstimator(
    //  kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, kRobotToCam);
        

     // we'll have to play around with this 
    //  photonEstimator13.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  @Override
  public void periodic() {

  }

  public PhotonPipelineResult getPipelineResult() {
    PhotonPipelineResult pipelineResult = camera.getLatestResult();
    return pipelineResult ;

  }


  public boolean getHasTarget() {


  var result = camera.getLatestResult();
  Boolean hasTar = result.hasTargets();

  
return hasTar ;


  }

// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/apriltagvision/AprilTagVision.java

 public double getAngle() {
    double angle;

  if (getHasTarget()) { 

  var result = camera.getLatestResult();
   angle = result.getBestTarget().getYaw();
  }

  else {

       angle = 0;
System.err.println("No Target");
    return 0;
  }

return angle ;

 }

 
 // this is the projected pose estimation from the camera
  //   public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
  //       var visionEst = photonEstimator13.update();
  //       double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
  //       boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
  //  // This is missing alot from the example that might or might not be important
  //       if (newResult) lastEstTimestamp = latestTimestamp;
      
  //       return visionEst;
  //   }



    public double getTimeStamp() {
        double timestamp = camera.getLatestResult().getTimestampSeconds();
return timestamp;
    }

//        public Pose3d getPose() {
//         Pose3d pose = photonEstimator13.getReferencePose();
// return pose;
//     }


    // this is some logic for rejecting bad poses
   public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = kSingleTagStdDevs;
        // var targets = getPipelineResult().getTargets();
        // int numTags = 0;
        // double avgDist = 0;
        // for (var tgt : targets) {
        //     var tagPose = photonEstimator13.getFieldTags().getTagPose(tgt.getFiducialId());
        //     if (tagPose.isEmpty()) continue;
        //     numTags++;
        //     avgDist +=
        //             tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        // }
        // if (numTags == 0) return estStdDevs;
        // avgDist /= numTags;
        // // Decrease std devs if multiple targets are visible
        // if (numTags > 1) estStdDevs = kMultiTagStdDevs;
        // // Increase std devs based on (average) distance
        // if (numTags == 1 && avgDist > 4)
        //     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        // else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;

      }









   // public void getRange() {
  //   var result = camera.getLatestResult();
  //   if (result.hasTargets()) {
  //     // First calculate range
  //     double range =
  //       PhotonUtils.calculateDistanceToTargetMeters(
  //         CAMERA_HEIGHT_METERS,
  //         TARGET_HEIGHT_METERS,
  //         CAMERA_PITCH_RADIANS,
  //         Units.degreesToRadians(result.getBestTarget().getPitch())); 
  //   }
  // }
}

