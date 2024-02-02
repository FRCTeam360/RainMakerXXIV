// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.IntakeIO.IntakeIOInputs;

public interface FlywheelIO {
  /** Creates a new FlywheelIO. */
  public static class FlywheelIOInputs {
    public double topSpeed = 0.0;
    public double bottomSpeed = 0.0;
  }
  
  public default void updateInputs(IntakeIOInputs inputs) {}

public void set(double speed);

public void setTop(double speed);

public void setBottom(double speed);

public void setReference(double rpm, ControlType kvelocity);

public void setTopReference(double rpm, ControlType kvelocity);

public void setBottomReference(double rpm, ControlType kvelocity);

public void stopTopMotor();

public void stopBottomMotor();

public double getTop();

public double getBottom();

public double getTopVelocity();

public double getBottomVelocity();

public double getBottom(double rpmSetpoint);

public double getPosition();

}
