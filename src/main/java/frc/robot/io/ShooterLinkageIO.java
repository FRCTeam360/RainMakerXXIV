// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ShooterLinkageIO {
  /** Creates a new ShooterLinkageIO. */
  @AutoLog
  public static class ShooterLinkageIOInputs {
    public double shooterLinkageAngle = 0.0;
  }

  public default void updateInputs(ShooterLinkageIOInputs inputs) {}
}
