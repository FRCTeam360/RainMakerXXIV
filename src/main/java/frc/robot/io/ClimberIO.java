// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ClimberIO {
  /** Creates a new ClimberIO. */
  @AutoLog
  public static class ClimberIOInputs {
    public double speedLeft = 0.0;
    public double speedRight = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public void setLeft(double speed);

  public void setRight(double speed);

}
