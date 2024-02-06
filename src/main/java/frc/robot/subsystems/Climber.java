// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.ClimberIO;
import frc.robot.io.ClimberIOInputsAutoLogged;
import frc.robot.io.IntakeIOInputsAutoLogged;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  /** Creates a new Climber. */
  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void run(double speed) {
    io.setRight(speed);
  }

  public void stop() {
    io.setLeft(0);
    io.setRight(0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
    // This method will be called once per scheduler run
  }
}
