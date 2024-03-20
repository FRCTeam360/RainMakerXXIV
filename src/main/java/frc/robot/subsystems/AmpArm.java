// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Objects;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.AmpArmIOTalonFX;
import frc.robot.io.AmpArmIO;
import frc.robot.io.AmpArmIOInputsAutoLogged;
import frc.robot.utils.CommandLogger;

public class AmpArm extends SubsystemBase {
  private AmpArmIO io;
  private final AmpArmIOInputsAutoLogged inputs = new AmpArmIOInputsAutoLogged();

  /** Creates a new AmpArm. */
  public AmpArm(AmpArmIO io) {
    this.io = io;
    SmartDashboard.putBoolean("Is arm at home", isArmAtZero());
    SmartDashboard.putBoolean("is wrist at home", isWristAtZero());
        SmartDashboard.putBoolean("amp intake sensor", getIntakeSensor());

    setupShuffleboard();
  }

  public void enableBrakeMode() {
    io.enableBrakeMode();
  }

  public void disableBrakeMode() {
    io.disableBrakeMode();
  }

  private boolean avoidWristCollision() {
    double armAngle = io.getArmPosition();
    double wristAngle = io.getWristPosition();
    // Added this boolean for readablity
    boolean safeFromCollision = false;
    if (armAngle > -10.0) {
      if (wristAngle > armAngle + 105.0) {
        io.setWrist(armAngle + 100.0);
        safeFromCollision = false;
      } else if (wristAngle < armAngle - 90.0) {
        io.setWrist(armAngle - 85.0);
        safeFromCollision = false;
      } else {
        safeFromCollision = true;
      }
    } else if (armAngle < -60.0) {
      io.setWrist(70.0);
      safeFromCollision = false;
    } else if (armAngle >= -60.0 && armAngle <= 0.0) {
      if (wristAngle > 70.0) {
        io.setWrist(70.0);
        safeFromCollision = false;
      } else if (wristAngle > 60.0) {

        io.setWrist(60.0);
        safeFromCollision = false;
      } else {
        safeFromCollision = true;
      }
    }
    return safeFromCollision;
  }

  /**
   * Used to avoid collision between the arm and the linkage
   * 
   * @param linkage Supplies the angle of the linkage
   * @return
   */
  private boolean avoidCollisionWithLinkage(Linkage linkage) {
    boolean safeFromCollision = false;
    if (Objects.isNull(linkage)) {
      safeFromCollision = true;
      return safeFromCollision;
    }

    double armAngle = io.getArmPosition();
    double linkageAngle = linkage.getAngle();

    if (armAngle > -10.0) {
      safeFromCollision = true;
    } else if (armAngle <= -73.5) {
      safeFromCollision = true;
    } else {
      if (linkageAngle > 5.0) {
        // Set the arm to the closest safe angle to prevent it from running into the
        // linkage
        if (armAngle < -37.5) {
          io.setArm(-74.0);
        } else {
          io.setArm(-6.0);
        }
        safeFromCollision = false;
      } else {
        safeFromCollision = true;
      }
    }
    return safeFromCollision;
  }

  /**
   * Avoid collisions between the arm and the linakge while running a position set
   * point
   * 
   * @param angle The angle to set the arm to
   */
  private boolean avoidCollisionWithLinkage(double angleSetpoint, Linkage linkage) {
    boolean safeFromCollision = true;
    if (Objects.isNull(linkage)) {
      safeFromCollision = true;
      return safeFromCollision;
    }
    double linkageAngle = linkage.getAngle();
    double armAngle = io.getArmPosition();

    if (linkageAngle > 5.0) {
      if (angleSetpoint < -6.5 && armAngle >= -6.5) {
        safeFromCollision = false;
        io.setArm(-6.0);
      } else if (angleSetpoint > -73.5 && armAngle <= -73.5) {
        safeFromCollision = false;
        io.setArm(-74.0);
      }
    }
    return safeFromCollision;

  }

  public void setArm(double angle, Linkage linkage) {
    boolean safeFromCollisionPower = avoidCollisionWithLinkage(linkage);
    boolean safeFromCollisionSetpoint = avoidCollisionWithLinkage(angle, linkage);
    if (safeFromCollisionPower && safeFromCollisionSetpoint) {
      avoidWristCollision();
      io.setArm(angle);
    } else {
      io.stopArm();
      io.stopWrist();
    }
  }

  public void setWrist(double angle) {
    if (avoidWristCollision()) {
      io.setWrist(angle);
    }
  }

  public void zeroWrist() {
    io.zeroWrist();
  }

  public void zeroArm() {
    io.zeroArm();
  }

  public boolean getIntakeSensor() {
    return io.getIntakeSensor();
  }

  public boolean isWristAtZero() {
    return Math.abs(getWristPosition()) < 2.0;
  }

  public boolean isArmAtZero() {
    return Math.abs(getArmPosition()) < 5.0;
  }

  public void runArm(double speed, Linkage linkage) {
    if (speed == 0) {
      io.stopArm();
    }
    if (avoidCollisionWithLinkage(linkage)) {
      avoidWristCollision();
      io.runArm(speed);
    }
  }

  public void runWrist(double speed) {
    if (avoidWristCollision()) {
      io.runWrist(speed);
    }
  }

  public void stopArm() {
    io.stopArm();
  }

  public void stopWrist() {
    io.stopWrist();
  }

  public void stopBoth() {
    io.stopWrist();
    io.stopArm();
  }

  public double getArmPosition() {
    return io.getArmPosition();
  }

  public double getWristPosition() {
    return io.getWristPosition();
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    tab.addNumber("Arm Angle", () -> this.getArmPosition());
    tab.addNumber("Wrist Angle", () -> this.getWristPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is arm at home", isArmAtZero());
    SmartDashboard.putBoolean("is wrist at home", isWristAtZero());
    SmartDashboard.putBoolean("amp intake sensor", getIntakeSensor());

    io.updateInputs(inputs);
    Logger.processInputs("AmpArm", inputs);

    if (io.getZeroButton()) {
      io.resetArmWristPos();
    }
    // io.enableBrakeMode();
    // if(RobotState.isDisabled()){ //TODO: FIX?
    // if(io.getBrakeButton()){
    // if(io.isBrakeMode()){
    // io.disableBrakeMode();
    // } else {
    // io.enableBrakeMode();
    // }
    // }
    // if (io.getZeroButton()) {
    // io.resetArmWristPos();
    // }
    // } else {
    // if(!io.isBrakeMode()){
    // io.enableBrakeMode();
    // }
    // }
  }

}
