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
    public double armSpeed = 0.0;
    public double wristSpeed = 0.0;
    public double armAngle = 0.0;
    public double wristAngle = 0.0;
    public double armAmps = 0.0;
    public double wristAmps = 0.0;
  }

  public default void updateInputs(AmpArmIOInputs inputs) {}

  public void runArm(double speed);

  public void runWrist(double speed);

  public void setArm(double angle);

  public void setWrist(double angle);

  public void stopArm();

  public void stopWrist();

  public double getArmPosition();

  public double getWristPosition();

  public void zeroWrist();

  public void zeroArm();
}
