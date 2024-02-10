// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface IntakeIO {
  /** Creates a new IntakeIO. */
  @AutoLog
  public static class IntakeIOInputs {
    public double intakeSpeed = 0.0;
    public double output = 0.0;
    public boolean sensor = true;
    public double amps = 0.0;
  }
  
  public default void updateInputs(IntakeIOInputs inputs) {}

  
  public void set(double speed);

  public void stopMotor();

  public double getPower();

  public double getOutputCurrent();

  public boolean getSideSensor();

  public boolean getHighSensor();
}
