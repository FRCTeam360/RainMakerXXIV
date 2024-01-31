// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.ShooterIO;
import frc.robot.io.ShooterIOInputsAutoLogged;
import frc.robot.io.ShooterIO.ShooterIOInputs;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  
  /** Creates a new Shooter.
  public Shooter(ShooterIO io) {
    this.io = io;
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

  public void stopLeft() {
    io.stopLeftMotor();
  }

  public void stopRight() {
    io.stopRightMotor();
  }

  public void stopBoth() {
    io.stopLeftMotor();
    io.stopRightMotor();
  }

  public double getLeftSpeed() {
    return io.getLeft();

  }

  public double getRightSpeed() {
    return io.getRight();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    SmartDashboard.putNumber("Left Speed", getLeftSpeed());
    SmartDashboard.putNumber("Right Speed", getRightSpeed());
  }
}
*/
