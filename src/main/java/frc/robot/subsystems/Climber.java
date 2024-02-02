// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.ClimberIO;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void run(double speed) {
    io.setLeft(speed);
  }

  public void stop() {
    io.setRight(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
