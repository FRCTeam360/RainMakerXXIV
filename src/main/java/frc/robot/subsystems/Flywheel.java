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

  double rpmSetpoint = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public void runTop(double speed) {
    io.setTop(speed);
  }

  public void runBottom(double speed) {
    io.setBottom(speed);
  }

  public void runBoth(double speed) {
    io.setTop(speed);
    io.setBottom(speed);
  }

  public void setTopRPM(double rpm) {
    io.setTopReference(rpm, ControlType.kVelocity);
  }

  public void setBottomRPM(double rpm) {
    io.setBottomReference(rpm, ControlType.kVelocity);
  }

  public void setBothRPM(double rpm) {
    io.setTopReference(rpm, ControlType.kVelocity);
    io.setBottomReference(rpm, ControlType.kVelocity);
  }

  public void stop() {
    io.stopTopMotor();
    io.stopBottomMotor();
  }

  public double getTopPower() {
    return io.getTopPower();
  }

  public double getBottomPower() {
    return io.getBottomPower();
  }

  public double getTopVelocity() {
    return io.getTopVelocity();
  }

  public double getBottomVelocity() {
    return io.getBottomVelocity();
  }

  public boolean isAtSetpoint() {
    return Math.abs(this.getTopVelocity() - rpmSetpoint) < 30.0;
  }

  public boolean isAboveSetpoint() {
    return this.getTopVelocity() >= rpmSetpoint;
  }

  public boolean isBelowSetpoint() {
    return this.getTopVelocity() <= rpmSetpoint - 30.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
    SmartDashboard.putNumber("Top Velocity", io.getTopVelocity());
    SmartDashboard.putNumber("Bottom Velocity", io.getBottomVelocity());
  }
}
