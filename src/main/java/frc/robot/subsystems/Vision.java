// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonUtils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.VisionIO;
import frc.robot.io.VisionIOInputsAutoLogged;
import frc.robot.io.VisionIO.VisionIOInputs;
import frc.robot.utils.CommandLogger;

public class Vision extends SubsystemBase {
  private final NetworkTable lime = NetworkTableInstance.getDefault().getTable("limelight");
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  /** Creates a new Limelight. */
  public Vision (VisionIO io) {
    this.io = io;
  }

  public double getTX() {
    return lime.getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return lime.getEntry("ty").getDouble(0);
  }

  public double getTV() {
    return lime.getEntry("tv").getDouble(0);
  }

  public double getPipeline() {
    return lime.getEntry("getpipe").getDouble(0);
  }

  public double[] getBotpose() {
    return lime.getEntry("botpose").getDoubleArray(new double[6]);
  }

  public void setPipeline(int pipeline) {
    lime.getEntry("pipeline").setNumber(pipeline);
  }
  
  public double getLinkageSetpoint() {
    return 0.0; // add linkage regression equation thing
  }

  public double getFlywheelSetpoint() {
    return 0.0; // add flywheel regression equation thing
  }
  
  // Returns true if the target is in view
  public boolean isTargetInView() {
    return getTV() == 1;
  }

  public double getDistanceBlueSourceRight1() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.22, .5235, 0.0);
  }

  public double getDistanceBlueSourceLeft2() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.22, .5235, 0.0);
  }

  public double getDistanceRedSpeakerRight3() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.32, .5235, 0.0);
  }

  public double getDistanceRedSpeakerCenter4() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.32, .5235, 0.0);
  }

  public double getDistanceRedAmp5() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.22, .5235, 0.0);
  }

  public double getDistanceBlueAmp6() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.22, .5235, 0.0);
  }

  public double getDistanceBlueSpeakerCenter7() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.32, .5235, 0.0);
  }

  public double getDistanceBlueSpeakerRight8() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.32, .5235, 0.0);
  }

  public double getDistanceRedSourceRight9() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.22, .5235, 0.0);
  }

  public double getDistanceRedSourceLeft10() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.22, .5235, 0.0);
  }

  public double getDistanceRedStage11() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.21, .5235, 0.0);
  }

  public double getDistanceRedStage12() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.21, .5235, 0.0);
  }

  public double getDistanceRedStage13() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.21, .5235, 0.0);
  }

  public double getDistanceBlueStage14() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.21, .5235, 0.0);
  }

  public double getDistanceBlueStage15() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.21, .5235, 0.0);
  }

  public double getDistanceBlueStage16() {
    return PhotonUtils.calculateDistanceToTargetMeters(0.3467, 1.21, .5235, 0.0);
  }

  @Override
  public void periodic() {
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      setPipeline(0);
    } else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      setPipeline(1);
    }
    // This method will be called once per scheduler run
  }
}
