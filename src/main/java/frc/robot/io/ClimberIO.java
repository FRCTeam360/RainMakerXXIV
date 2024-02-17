// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  /** Creates a new ClimberIO. */
  @AutoLog
  public static class ClimberIOInputs {
    public double speedLeft = 0.0;
    public double speedRight = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {
  }

  public void runLeft(double speed);

  public void runRight(double speed);

  public void runBoth(double leftSpeed, double rightSpeed);

  public void stop();

  public boolean leftAboveMinHeight();

  public boolean rightAboveMinHeight();

  public double getLeftPosition();

  public double getRightPosition();

  public void zeroBoth();
}
