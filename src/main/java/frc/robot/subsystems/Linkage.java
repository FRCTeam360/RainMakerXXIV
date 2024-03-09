// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import java.util.Objects;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.LinkageIO;
import frc.robot.io.LinkageIOInputsAutoLogged;
import frc.robot.utils.CommandLogger;

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
    tab.addDouble("Angle", () -> this.getAngle());
    tab.addBoolean("Brake Mode", () -> io.isBrakeMode());

    SmartDashboard.putNumber("Linkage Angle", getAngle());
    SmartDashboard.putBoolean("Linkage is at home", isAtZero());
  }

  public boolean isAtSetpoint() {
	  if(Math.abs(this.getAngle() - positionSetpoint) < 3.0) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Avoids collision between the linkage and the amp arm
   * @param AmpArm ampArm
   */
  private boolean avoidCollisionWithAmpArm(AmpArm ampArm) {
    boolean safeFromCollision = false;
    if (Objects.isNull(ampArm)){
      safeFromCollision = true;
      return safeFromCollision;
    }
    double armAngle = ampArm.getArmPosition();
    double linkageAngle = io.getPosition();
    if (armAngle > -6.5) {
      safeFromCollision = true;
    } else if (armAngle < -70.0) {
      safeFromCollision = true;
    } else {
      io.setReference(0.0);
      safeFromCollision = false;
    }
    return safeFromCollision;
  }

  public void run(double speed, AmpArm ampArm) {
    if (avoidCollisionWithAmpArm(ampArm)) {
      io.set(speed);
    }
  }

  public void stop() {
    io.stopMotor();
  }

  public double getAngle() {
    return io.getPosition();
  }

  public void setAngle(double setPoint, AmpArm ampArm) {
    if (avoidCollisionWithAmpArm(ampArm)) {
      io.setReference(setPoint);
      positionSetpoint = setPoint;
    } else {
      io.stopMotor();
    }
  }

  public double getPower() {
    return io.get();
  }

  public void zero() {
    io.setPosition(0.0); //-10.5 when resting on hard stops
  }

  public void setEncoderTo174() {
    io.setPosition(174.0);
  }
  public void enableBrakeMode(){
    io.enableBrakeMode();
  }

  public void disableBrakeMode() {
    io.disableBrakeMode();
  }

  public void setFFWScaling(double ff) {
    io.setFF(ff * Math.cos(getAngle()));
  }

  public boolean isAtZero() {
    return Math.abs(getAngle()) < 5.0;
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Linkage Angle", getAngle());
    SmartDashboard.putBoolean("Linkage is at home", isAtZero());
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Linkage", inputs);

    if(RobotState.isDisabled()){
      if(io.getBrakeButton()){
        if(io.isBrakeMode()){
          io.disableBrakeMode();
        } else {
          io.enableBrakeMode();
        }
      }
      if (io.getZeroButton()) {
        this.zero();
      }
    } else {
      if(!io.isBrakeMode()){
        io.enableBrakeMode();
      }
      // DO NOT REMOVE
      // This is neccessary to run the linkage after playing the update sound
      io.stopSound();
    }
  }
}
