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

  private static Flywheel instance;

  private final CANSparkFlex topMotor = new CANSparkFlex(Constants.FLYWHEEL_LEFT_ID, MotorType.kBrushless);
  private final RelativeEncoder topEncoder = topMotor.getEncoder();
  private final SparkPIDController topPIDController = topMotor.getPIDController();

  private final CANSparkFlex bottomMotor = new CANSparkFlex(Constants.FLYWHEEL_RIGHT_ID, MotorType.kBrushless);
  private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
  private final SparkPIDController bottomPIDController = bottomMotor.getPIDController();

  private double topP = 0.00055;
  private double topI = 0.0;
  private double topD = 0.0;
  private double topFF = 0.000158;

  private double bottomP = 0.00055;
  private double bottomI = 0.0;
  private double bottomD = 0.0;
  private double bottomFF = 0.000155;

  private double rpmSetpoint = 0.0;

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

  public boolean isAboveSetpoint(double setpoint) {
    return this.getTopVelocity() >= setpoint;
  }

  public boolean isBelowSetpoint() {
    return this.getTopVelocity() <= rpmSetpoint - 30.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }
}
