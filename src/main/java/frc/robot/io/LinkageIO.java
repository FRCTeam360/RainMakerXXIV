// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface LinkageIO {
  /** Creates a new LinkageIO. */
  @AutoLog
  public static class LinkageIOInputs {
    public double linkageAngle = 0.0;
    public double linkageVoltage = 0.0;
  }

  public default void updateInputs(LinkageIOInputs inputs) {}

  public default void set(double speed) {}

  public void stopMotor();

  public double getPosition();

  public double get();

  public void setFF(double d);

  public double getAppliedOutput();

  public void setReference(int setPoint, ControlType kposition);

  public void setPosition(double angle);
}
