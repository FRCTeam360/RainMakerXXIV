// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.AmpArmIO.AmpArmIOInputs;

public interface AmpIntakeIO {

  /** Creates a new AmpIntakeIO. */
  @AutoLog
  public static class AmpIntakeIOInputs {
    public double ampIntakeStatorCurrent = 0.0;
    public double ampIntakeVoltage = 0.0;
    public double ampIntakeVelocity = 0.0;
    public double ampIntakePosition = 0.0;
  }

  public default void updateInputs(AmpIntakeIOInputs inputs) {}

  public void runIntake(double speed);

  public void stop();

  public double getEncoderPosition();

  public double getIntakeSpeed();

  public double getAmps();

}
