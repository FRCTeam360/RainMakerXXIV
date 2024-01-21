// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.ShooterLinkageIO;
import frc.robot.io.ShooterLinkageIOInputsAutoLogged;
import frc.robot.io.ShooterLinkageIO.ShooterLinkageIOInputs;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterLinkage extends SubsystemBase {
  private final ShooterLinkageIO io;
  private final ShooterLinkageIOInputsAutoLogged inputs = new ShooterLinkageIOInputsAutoLogged();

  /** Creates a new ShooterLinkage. */
  public ShooterLinkage(ShooterLinkageIO io) {
    this.io = io;
  }

  public void run(double speed) {
    io.set(speed);
  }

  public void stop() {
    io.stopMotor();
  }

  public double getAngle() {
    return io.getPosition();
  }

  public double getSpeed() {
    return io.get();
  }

  public void zero() {
    io.setPosition(0);
  }

  public void setFFWScaling(double ff) {
    io.setFF(ff * Math.cos(getAngle()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("ShooterLinkage", (LoggableInputs) inputs);
    SmartDashboard.putNumber("Linkage Angle", getAngle());
    SmartDashboard.putNumber("linkage voltage", io.getAppliedOutput());
  }

}
