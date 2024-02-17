// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.ClimberIO;
import frc.robot.io.ClimberIOInputsAutoLogged;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public double heightOffset = 0;

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;

    SmartDashboard.putNumber("Left Height", 0);
    SmartDashboard.putNumber("Right Height", 0);
    SmartDashboard.putNumber("roll", 0);
    SmartDashboard.putNumber("height offset", heightOffset);
  }

  public void runBoth(double leftSpeed, double rightSpeed) {
    io.runBoth(leftSpeed, rightSpeed);
  }

  public void runLeft(double speed) {
    io.runLeft(speed);
  }

  public void runRight(double speed) {
    io.runRight(speed);
  }

  public void stop() {
    io.runLeft(0);
    io.runRight(0);
  }

  public void level() {
    io.level();
  }

  public boolean leftAboveMinHeight() {
    return io.leftAboveMinHeight();
  }

  public boolean rightAboveMinHeight() {
    return io.rightAboveMinHeight();
  }

  public double getLeftPosition() {
    return io.getLeftPosition();
  }

  public double getRightPosition() {
    return io.getRightPosition();
  } 

  public double getRoll() {
    return io.getRoll();
  }

  public void zeroBoth() {
    io.zeroBoth();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    SmartDashboard.putNumber("Left Height", getLeftPosition());
    SmartDashboard.putNumber("Right Height", getRightPosition());
    SmartDashboard.putNumber("roll", getRoll());
    SmartDashboard.putNumber("height offset", heightOffset);
  }
}
