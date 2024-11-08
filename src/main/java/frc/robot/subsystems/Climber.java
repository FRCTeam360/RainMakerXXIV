// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
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
import frc.robot.io.IntakeIOInputsAutoLogged;
import frc.robot.utils.CommandLogger;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;

    SmartDashboard.putNumber("Position L", getLeftPosition());
    SmartDashboard.putNumber("Position R", getRightPosition());

    zeroBoth();
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

  public void zeroBoth() {
    io.zeroBoth();
  }

  public void setLeftHeight(double height, int pidSlot) {
    io.setLeftHeight(height, pidSlot);
  }

  public void setRightHeight(double height, int pidSlot) {
    io.setRightHeight(height, pidSlot);
  }

  public void updatePIDF(double P, double I, double D, double F) {
    io.updatePIDF(P, I, D, F);
  }

  @Override public void periodic() {
    SmartDashboard.putNumber("Position L", getLeftPosition());
    SmartDashboard.putNumber("Position R", getRightPosition());
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    SmartDashboard.putNumber("Left Height", getLeftPosition());
    SmartDashboard.putNumber("Right Height", getRightPosition());
    //SmartDashboard.putNumber("roll", getRoll());
    //SmartDashboard.putNumber("height offset", heightOffset);
  }
}
