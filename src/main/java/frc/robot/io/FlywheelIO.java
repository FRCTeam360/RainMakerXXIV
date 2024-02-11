// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.IntakeIO.IntakeIOInputs;

public interface FlywheelIO {
  /** Creates a new FlywheelIO. */
  @AutoLog
  public static class FlywheelIOInputs {
    public double topSpeed = 0.0;
    public double bottomSpeed = 0.0;
  }
  
  public default void updateInputs(FlywheelIOInputs inputs) {}

public void setLeft(double speed);

public void setRight(double speed);

public void setLeftReference(double rpm, ControlType kvelocity);

public void setRightReference(double rpm, ControlType kvelocity);

public void stopLeftMotor();

public void stopRightMotor();

public double getLeftPower();

public double getRightPower();

public double getLeftVelocity();

public double getRightVelocity();

}
