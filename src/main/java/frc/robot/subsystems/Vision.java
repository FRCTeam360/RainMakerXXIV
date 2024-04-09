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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.io.VisionIO;
import frc.robot.io.VisionIOInputsAutoLogged;
import frc.robot.io.VisionIO.VisionIOInputs;
import frc.robot.utils.CommandLogger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private Timer snapshotTimer = new Timer();
  private Timer blinkTimer = new Timer();

  private final String VISION_LOGGING_PREFIX = "Vision: ";

  /** Creates a new Limelight. */
  public Vision(VisionIO io) {
    this.io = io;
  }

  public void blink() {
    io.setLEDMode(2);
    blinkTimer.stop();
    blinkTimer.reset();
    blinkTimer.start();
  }

  public void lightsOn() {
    io.setLEDMode(3);
  }

  public void lightsOut() {
    io.setLEDMode(1);
    blinkTimer.stop();
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
    double ty = this.getTY();
    double fudgeFactor = -1.5;
    //return (-0.000182*Math.pow(ty, 4)+0.000622*Math.pow(ty, 3)+0.039998*Math.pow(ty, 2)+0.944848*(ty)+lastBit); //pre sammamish
    return (0.000441259 * Math.pow(ty, 3) + -0.021738 * Math.pow(ty, 2) + 0.953749 * ty + 163.092 + fudgeFactor); // before worlds !!
  }

  public double getFlywheelSetpoint() {
    if(io.getTYBase() <-9.0) {
      return 8500.0;
    }  else {
      return 7500.0;
    }
  }

  public void takeSnapshot() {
    io.takeSnapshot();
    Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", true);
    snapshotTimer.stop();
    snapshotTimer.reset();
    snapshotTimer.start();
  }

  public void resetSnapshot() {
    io.resetSnapshot();
    Logger.recordOutput(VISION_LOGGING_PREFIX + "snapshot", false);
    snapshotTimer.stop();
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
    if(snapshotTimer.get() > 0.2){
      resetSnapshot();
    }
    if(blinkTimer.get() > 0.5) {
      lightsOut();
    }
    io.updateInputs(inputs);
    Logger.processInputs("Limelight", inputs);
    SmartDashboard.putNumber("tY", io.getTYBase());
    // This method will be called once per scheduler run
  }
}
