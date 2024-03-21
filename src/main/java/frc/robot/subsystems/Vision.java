// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import java.util.Objects;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.VisionIO;
import frc.robot.io.VisionIOInputsAutoLogged;
import frc.robot.io.VisionIO.VisionIOInputs;
import frc.robot.utils.CommandLogger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  /** Creates a new Limelight. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  public void blink() {
    io.setLEDMode(2);
  }

  public void lightsOut() {
    io.setLEDMode(1);
  }

  public double getTX() {
    return io.getTX();
  }

  public double getTY() {
    return io.getTYAdjusted();
  }

  public double getTV() {
    return io.getTV();
  }

  public double getPipeline() {
    return io.getPipeline();
  }

  public Pose2d getBotPose() {
    return io.getBotPose();
  }

  public void setPipeline(int pipeline) {
    if (io.getPipeline() != pipeline) {
      io.setPipeline(pipeline);
    }
  }

  public double getLinkageSetpoint() {
    return 0.0; // add linkage regression equation thing
  }

  public double getFlywheelSetpoint() {
    return 0.0; // add flywheel regression equation thing
  }

  public void takeSnapshot() {
    io.takeSnapshot();
  }

  public boolean isOnTargetTX() {
    if (Math.abs(getTX()) < 3.0) {
      return true;
    }
    return false;
  }

  // Returns true if the target is in view
  public boolean isTargetInView() {
    return getTV() == 1;
  }

  @Override
  public void periodic() {
    if (DriverStation.isDSAttached()) {
      if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        setPipeline(0);
      } else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        setPipeline(1);
      }
    }
    io.updateInputs(inputs);
    Logger.processInputs("Limelight", inputs);
    // This method will be called once per scheduler run
  }
}
