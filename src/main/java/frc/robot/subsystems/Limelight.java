// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final VisionIO[] io;
  //NetworkTable lime = NetworkTableInstance.getDefault().getTable("limelight");
  private static final double LOWEST_DISTANCE = Units.feetToMeters(10.0);
  private final VisionIOInputsAutoLogged[] inputs;

  private int acceptableID;
  private boolean useSingleTag = false;


  private final List<Limelight.PoseAndTimestamp> results = new ArrayList<>();
  /** Creates a new Limelight. */
  public Limelight(VisionIO[] io) {
    this.io = io;
    inputs = new VisionIOInputsAutoLogged[io.length];

    for (int i = 0; i < io.length; i++) {
        inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  // public double getTX() {
  //   return lime.getEntry("tx").getDouble(0);
  // }

  // public double getTY() {
  //   return lime.getEntry("ty").getDouble(0);
  // }

  // public double getTV() {
  //   return lime.getEntry("tv").getDouble(0);
  // }

  @Override
  public void periodic() {
    Logger.recordOutput("useSingleTag", useSingleTag);
    results.clear();
    for(int i = 0; i < inputs.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + io[i].getName() + "/Inputs", inputs[i]);

      if (inputs[i].hasTarget
              && inputs[i].isNew
              && !DriverStation.isAutonomous()
              && inputs[i].maxDistance < LOWEST_DISTANCE) {
          if (useSingleTag) {
              if (inputs[i].singleIDUsed == acceptableID) {
                  processVision(i);
              }
          } else {
              processVision(i);
          }
      }
    }
    Logger.recordOutput("Vision/ResultCount", results.size());
    // This method will be called once per scheduler run
  }

  public List<Limelight.PoseAndTimestamp> getVisionOdometry() {
    return results;
  }

  public static class PoseAndTimestamp {
    Pose2d pose;
    double timestamp;

    public PoseAndTimestamp(Pose2d pose, double timestamp) {
      this.pose = pose;
      this.timestamp = timestamp;
    }

    public Pose2d getPose() {
      return pose;
    }

    public double getTimestamp() {
      return timestamp;
    }
  }

  public void setUseSingleTag(boolean useSingleTag) {
    setUseSingleTag(useSingleTag, 0);
  }

  public void setUseSingleTag(boolean useSingleTag, int accpetableID) {
    this.useSingleTag = useSingleTag;
    this.acceptableID = accpetableID;
  }

  public void setReferencePose(Pose2d pose) {
    for(VisionIO io : io) {
      io.setReferencePose(pose);
    }
  }

  public double getMinDistance(int camera) {
    return inputs[camera].minDistance;
  }

  public void processVision(int cameraNum) {
    Pose2d currentPose =
        new Pose2d(inputs[cameraNum].x, inputs[cameraNum].y, new Rotation2d(inputs[cameraNum].rotation));
        Logger.recordOutput(io[cameraNum].getName() + " pose", currentPose);

        // add the new pose to a list
        results.add(new PoseAndTimestamp(currentPose, inputs[cameraNum].timestamp));
  }
}