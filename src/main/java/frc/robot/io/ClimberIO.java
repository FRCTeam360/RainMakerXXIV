// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  /** Creates a new ClimberIO. */
  @AutoLog
  public static class ClimberIOInputs {
    public double climberSpeedLeft = 0.0;
    public double climberSpeedRight = 0.0;
    public double climberLeftAmps = 0.0;
    public double climberRightAmps = 0.0;
    public double climberLeftVoltage = 0.0;
    public double climberRightVoltage = 0.0;
    public double climberLeftVelocity = 0.0;
    public double climberRightVelocity = 0.0;
    public double climberLeftPosition = 0.0;
    public double climberRightPosition = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {
  }

  public void runLeft(double speed);

  public void runRight(double speed);

  public void runBoth(double leftSpeed, double rightSpeed);

  public void stop();

  public void level();

  public boolean leftAboveMinHeight();

  public boolean rightAboveMinHeight();

  public double getLeftPosition();

  public double getRightPosition();

  public double getRoll();

  public void zeroBoth();

  public void setLeftHeight(double goalHeight);

  public void setRightHeight(double goalHeight);

  public void updatePIDF(double kP, double kI, double kD, double kFF);
}
