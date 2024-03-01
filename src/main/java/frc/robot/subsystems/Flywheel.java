// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.FlywheelIO;
import frc.robot.io.FlywheelIOInputsAutoLogged;
import frc.robot.io.IntakeIOInputsAutoLogged;
import frc.robot.utils.CommandLogger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private double rpmSetpoint = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
    //SmartDashboard.putNumber("error", 0);
    SmartDashboard.putBoolean("Is At Setpoint", false);
  }

  public void runLeft(double speed) {
    io.setLeft(speed);
  }

  public void runRight(double speed) {
    io.setRight(speed);
  }

  public void runBoth(double leftSpeed, double rightSpeed) {
    io.setLeft(leftSpeed);
    io.setRight(rightSpeed);
  }

  public void setLeftRPM(double rpm) {
    rpmSetpoint = rpm;
    io.setLeftReference(rpm, ControlType.kVelocity);
  }

  public void setRightRPM(double rpm) {
    rpmSetpoint = rpm;
    io.setRightReference(rpm, ControlType.kVelocity);
  }

  public void handoff(double rpm) {
    rpmSetpoint = rpm;
    io.setLeftReference(rpm, ControlType.kVelocity);
    io.setRightReference(rpm, ControlType.kVelocity);

  }

  public void setBothRPM(double rpm) {
    rpmSetpoint = rpm;
    if (rpm > 500) {
      io.setLeftReference(rpm, ControlType.kVelocity);
      if (rpm > 6500) {
        rpm = rpm - 750;
      }
      io.setRightReference(rpm, ControlType.kVelocity);
    } else {
      stop();
    }
  }

  public void stop() {
    io.stopLeftMotor();
    io.stopRightMotor();
  }

  public double getLeftPower() {
    return io.getLeftPower();
  }

  public double getRightPower() {
    return io.getRightPower();
  }

  public double getLeftVelocity() {
    return io.getLeftVelocity();
  }

  public double getRightVelocity() {
    return io.getRightVelocity();
  }

  // public boolean topIsAtSetpoint() {
  // return Math.abs(this.getTopVelocity() - topRPMSetpoint) < 30.0;
  // }

  // public boolean bottomIsAtSetpoint() {
  // return Math.abs(this.getBottomVelocity() - bottomRPMSetpoint) < 30.0;
  // }

  // public boolean areBothAtSetpoint() {
  // return bottomIsAtSetpoint() && topIsAtSetpoint();
  // }

  // public boolean isAboveSetpoint() {
  // return this.getTopVelocity() >= topRPMSetpoint;
  // }

  // public boolean isBelowSetpoint() {
  // return this.getTopVelocity() <= topRPMSetpoint - 30.0; }
  public boolean isAtSetpoint() {
    return Math.abs(this.getLeftVelocity() - rpmSetpoint) < 175.0;
  }

  public boolean isAboveSetpoint() {
    return this.getLeftVelocity() >= rpmSetpoint;
  }

  public boolean isBelowSetpoint() {
    return this.getLeftVelocity() <= rpmSetpoint - 30.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Is At Setpoint", isAtSetpoint());
    // SmartDashboard.putNumber("setpoint rpm", rpmSetpoint);
    // SmartDashboard.putNumber("curren left rpm", getLeftVelocity());
    // SmartDashboard.putNumber("current right rpm", getRightVelocity());
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.recordOutput("Flywheel Command",
        Objects.isNull(getCurrentCommand()) ? "null" : getCurrentCommand().getName());
    Logger.processInputs("Flywheel", inputs);
  }
}
