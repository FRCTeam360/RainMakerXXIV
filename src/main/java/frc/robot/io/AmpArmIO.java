// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface AmpArmIO {
  /** Creates a new AmpArmIO. */
  @AutoLog
  public static class AmpArmIOInputs {
    public double armStatorCurrent = 0.0;
    public double wristStatorCurrent = 0.0;
    public double armSupplyCurrent = 0.0;
    public double wristSupplyCurrent = 0.0;
    public double armVoltage = 0.0;
    public double wristVoltage = 0.0;
    public double armVelocity = 0.0;
    public double wristVelocity = 0.0;
    public double armPosition = 0.0;
    public double wristPosition = 0.0;
  }

  public default void updateInputs(AmpArmIOInputs inputs) {}

  public void runArm(double speed);

  public void runWrist(double speed);

  public void stopArm();

  public void stopWrist();

  public double getArmPosition();

  public double getWristPosition();
}
