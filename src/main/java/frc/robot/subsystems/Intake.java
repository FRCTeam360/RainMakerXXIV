// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.io.IntakeIO;
import frc.robot.io.IntakeIOInputsAutoLogged;
import frc.robot.io.IntakeIO.IntakeIOInputs;
import frc.robot.utils.CommandLogger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public void setupShuffleboard() {
    ShuffleboardTab tab = Shuffleboard.getTab("intake");
    tab.addNumber("Encoder position", () -> io.getEncoderValue());
  }
  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
    setupShuffleboard();
  }

  public boolean isAtEncoderSetpoint(double setpoint) {
    return Math.abs(getEncoderValue() - setpoint) < 0.005 ? true : false;
  }

  public boolean getSideSensor() {
    return io.getSideSensor();
  }

  public boolean getHighSensor() {
    return io.getHighSensor();
  }

  public void run(double speed) {
    io.set(speed);
  }

  public void stop() {
    io.stopMotor();
  }

  public double getPower() {
    return io.getPower();
  }

  public double getAmps() {
    return io.getOutputCurrent();
  }

  public double getEncoderValue() {
    return io.getEncoderValue();
  }

  public void setEncoderValue(double encoderPosition) {
    io.setEncoderValue(encoderPosition);
  }

  public void moveEncoder(double setpoint) {
    io.moveEncoder(setpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    CommandLogger.logCommandSubsystem(this);
  }
}
