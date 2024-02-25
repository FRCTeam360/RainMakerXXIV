// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.AmpIntakeIO;
import frc.robot.io.AmpIntakeIOInputsAutoLogged;
import frc.robot.utils.CommandLogger;

public class AmpIntake extends SubsystemBase {
  private AmpIntakeIO io;
  private final AmpIntakeIOInputsAutoLogged inputs = new AmpIntakeIOInputsAutoLogged();

  /** Creates a new AmpIntake. */
  public AmpIntake(AmpIntakeIO io) {
    this.io = io;
    setupShuffleboard();
  }

  public void runIntake(double speed){
    io.runIntake(speed);
  }

  public void stop() {
    io.stop();
  }

  public double getAmps() {
    return io.getAmps();
  }

  public double getIntakeSpeed() {
    return io.getIntakeSpeed();
  }

  private void setupShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    tab.addNumber("Intake Speed", () -> this.getIntakeSpeed());
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("rotations", io.getEncoderPosition());
    SmartDashboard.putNumber("arm intake amps", io.getAmps());
    io.updateInputs(inputs);
    Logger.processInputs("AmpIntake", inputs);
  }
}
