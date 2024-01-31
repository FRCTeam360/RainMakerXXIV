// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.io.IntakeIO;
import frc.robot.io.IntakeIOInputsAutoLogged;
import frc.robot.io.IntakeIO.IntakeIOInputs;

import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

<<<<<<< HEAD
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Intake. */
  public Intake(IntakeIO io) {
    this.io = io;
=======
  private static Intake instance;
  private final DigitalInput sideSensor = new DigitalInput(2);
  private final DigitalInput highSensor = new DigitalInput(0);
  private final CANSparkMax motor = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);
  public final RelativeEncoder encoder = motor.getEncoder();

  /** Creates a new Intake. */
  public Intake() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kCoast);
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  public boolean getSideSensor() {
    return sideSensor.get();
  }

  public boolean getHighSensor() {
    return highSensor.get();
>>>>>>> Woodbot
  }

  public void run(double speed) {
    io.set(speed);
  }

  public void stop() {
    io.stopMotor();
  }

  public double getSpeed() {
    return io.get();
  }

  public boolean getSensor() {
    return io.getSensor();
  }

  public double getAmps() {
    return io.getOutputCurrent();
  }

  // public void setEncoder() {
  //   encoder.setPosition(encoder.getPosition() + 1.28436279297);
  // }

  @Override
  public void periodic() {
<<<<<<< HEAD
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    SmartDashboard.putNumber("Intake Speed", getSpeed());
    SmartDashboard.putNumber("Amps", getAmps());
    SmartDashboard.putBoolean("this sensor sucks", getSensor());
=======
    // SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
    // SmartDashboard.getNumber("Encoder Position", encoder.getPosition());
    // // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Intake Speed", getSpeed());
    // SmartDashboard.putNumber("Intake Amps", getAmps());
>>>>>>> Woodbot
  }
}
