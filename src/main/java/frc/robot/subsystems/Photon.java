// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Photon extends SubsystemBase {
  /** Creates a new Photon. */
  public final String kCameraName = "lime_13"  ;
  private final PhotonCamera camera;


  //TODO: FIX THESE NUMBERS
  private final double CAMERA_HEIGHT_METERS = 0.25  ;
  private final double TARGET_HEIGHT_METERS = 0.25 ;
  private final double CAMERA_PITCH_RADIANS = 0.0;

  public Photon() {
    camera = new PhotonCamera(kCameraName);
  }

  @Override
  public void periodic() {}

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  public void getRange() {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      // First calculate range
      double range =
        PhotonUtils.calculateDistanceToTargetMeters(
          CAMERA_HEIGHT_METERS,
          TARGET_HEIGHT_METERS,
          CAMERA_PITCH_RADIANS,
          Units.degreesToRadians(result.getBestTarget().getPitch())); 
    }
  }

  public double getAngle() {
    return getLatestResult().getBestTarget().getYaw();
  }
}
