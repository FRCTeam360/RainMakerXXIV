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

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

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
    SmartDashboard.putNumber("error", 0);
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
    io.setLeftReference(rpm, ControlType.kVelocity);
  }

  public void setRightRPM(double rpm) {
    io.setRightReference(rpm, ControlType.kVelocity);
  }

  public void setBothRPM(double rpm) {
    io.setLeftReference(rpm, ControlType.kVelocity);
    io.setRightReference(rpm, ControlType.kVelocity);
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

  public boolean isAtSetpoint() {
    return Math.abs(this.getLeftVelocity() - rpmSetpoint) < 30.0;
  }

  public boolean isAboveSetpoint(double setpoint) {
    return this.getLeftVelocity() >= setpoint;
  }

  public boolean isBelowSetpoint() {
    return this.getLeftVelocity() <= rpmSetpoint - 30.0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("setpoint rpm", rpmSetpoint);
    SmartDashboard.putNumber("curren left rpm", getLeftVelocity());
    SmartDashboard.putNumber("current right rpm", getRightVelocity());
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }
}
