// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ShooterIO {
  @AutoLog
  /** Creates a new ShooterIO. */
  public static class ShooterIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double leftSpeed = 0.0;
    public double rightSpeed = 0.0;
  }
  
  public default void updateInputs(ShooterIOInputs inputs) {}

  public void stopLeftMotor();

  public void stopRightMotor();

  public double getLeft();

  public double getRight();

  public void setLeft(double leftSpeed);

  public void setRight(double speed); 
}
