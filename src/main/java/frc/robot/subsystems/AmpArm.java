// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.AmpArmIO;
import frc.robot.io.AmpArmIOInputsAutoLogged;
import frc.robot.utils.CommandLogger;

public class AmpArm extends SubsystemBase {
  private AmpArmIO io;
  private final AmpArmIOInputsAutoLogged inputs = new AmpArmIOInputsAutoLogged();

  /** Creates a new AmpArm. */
  public AmpArm(AmpArmIO io) {
    this.io = io;
    setupShuffleboard();
  }

  public void runArm(double speed) {
    io.runArm(speed);
  }

  public void runWrist(double speed) {
    io.runWrist(speed);
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
    io.updateInputs(inputs);
    Logger.processInputs("AmpArm", inputs);
    CommandLogger.logCommandSubsystem(this);
    setupShuffleboard();
  }

}
