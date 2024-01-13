// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.IntakeIOInputsAutoLogged;
import frc.robot.io.ShooterIO;
import frc.robot.io.ShooterIOInputsAutoLogged;
import frc.robot.io.ShooterIO.ShooterIOInputs;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;


public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  
  private final CANSparkMax lead = new CANSparkMax(Constants.SHOOTER_LEAD_ID, MotorType.kBrushless);
  private final CANSparkMax follow = new CANSparkMax(Constants.SHOOTER_FOLLOW_ID, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    this.io = io;

    lead.restoreFactoryDefaults();
    lead.setInverted(false);
    lead.setIdleMode(IdleMode.kBrake);

    follow.restoreFactoryDefaults();
    follow.setInverted(true);
    follow.setIdleMode(IdleMode.kBrake);
    follow.follow(lead);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
  
  public void run(double speed) {
    lead.set(speed);
  }
}
