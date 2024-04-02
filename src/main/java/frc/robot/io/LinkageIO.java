// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface LinkageIO {
  /** Creates a new LinkageIO. */
  @AutoLog
  public static class LinkageIOInputs {
    public double linkageVoltage = 0.0;
    public double linkagePosition = 0.0;
    public double linkageVelocity = 0.0;
    public double linkageStatorCurrent = 0.0;
    public double linkageSupplyCurrent =  0.0;
    public boolean zeroButton = false;
    public boolean brakeButton = false;
    public double linkageSetpoint = 0.0;
    public boolean linkageZeroed = false;
  }

  public default void updateInputs(LinkageIOInputs inputs) {}

  public default void set(double speed) {}

  public void stopMotor();

  public double getPosition();

  public double get();

  public void setFF(double d);

  public double getAppliedOutput();

  public void setReference(double setPoint);

  public void setPosition(double angle);

  public double getVelocity();

  public void enableBrakeMode();

  public void disableBrakeMode();

  public boolean isBrakeMode();

  public boolean getZeroButton();

  public boolean getBrakeButton();

  /**
   * Stops playing sound on the linkage, this is neccessary to run the linkage
   */
  public void stopSound();
}
