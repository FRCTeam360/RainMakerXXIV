// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

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
  }

  public void run(double speed) {
    motor.set(speed);
  }
  
  public void stop() {
    motor.stopMotor();
  }

  public double getAmps() {
    return motor.getOutputCurrent();
  }

  public double getPower() {
    return motor.get();
  }

  // public void setEncoder() {
  //   encoder.setPosition(encoder.getPosition() + 1.28436279297);
  // }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Encoder Position", encoder.getPosition());
    // SmartDashboard.getNumber("Encoder Position", encoder.getPosition());
    // // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Intake Speed", getSpeed());
    // SmartDashboard.putNumber("Intake Amps", getAmps());
  }
}
