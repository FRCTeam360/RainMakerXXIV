// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.LinkageIO;
import frc.robot.io.LinkageIOInputsAutoLogged;

public class Linkage extends SubsystemBase {
  private final LinkageIO io;
  private final LinkageIOInputsAutoLogged inputs = new LinkageIOInputsAutoLogged();
  private double positionSetpoint;
  private static final double STARTING_ANGLE = 50.0;
  static XboxController driverCont = new XboxController(0);

  
  /** Creates a new ShooterLinkage. */
  public Linkage(LinkageIO io) {
  this.io = io;
    ShuffleboardTab tab = Shuffleboard.getTab("Linkage");
    tab.addBoolean("Zero Button", () -> io.getZeroButton());
    tab.addBoolean("Brake Button", () -> io.getBrakeButton());
  }

  public boolean isAtSetpoint() {
	  return false;
  }

  public void run(double speed) {
    System.out.println("linkage speed is " + speed);
    io.set(speed);
  }

  public void stop() {
    io.stopMotor();
  }

  public double getAngle() {
    return io.getPosition();
  }

  public void setAngle(double setPoint){
    io.setReference(setPoint);
    positionSetpoint = setPoint;
    io.setReference(setPoint);
  }

  public double getPower() {
    return io.get();
  }

  public void zero() {
    io.setPosition(0);
  }

  public void setEncoderTo90() {
    io.setPosition(90);
  }

  public void setFFWScaling(double ff) {
    io.setFF(ff * Math.cos(getAngle()));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Linkage", inputs);
  }
}
