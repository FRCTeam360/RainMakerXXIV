// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.IntakeIO;
import frc.robot.Constants;

public class IntakeIOSparkMax implements IntakeIO {
  /** Creates a new IntakeIOSparkMax. */
    private final CANSparkMax sparkMax = new CANSparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder encoder = sparkMax.getEncoder();
    private final SparkPIDController pid = sparkMax.getPIDController();
    // new hardware class for sensor?
    private final DigitalInput sideSensor = new DigitalInput(Constants.SIDE_SENSOR_PORT); // update port later idk what it is
    private final DigitalInput shooterSensor = new DigitalInput(Constants.SHOOTER_SENSOR_PORT); // update port later idk what it is
    private final DigitalInput intakeSensor = new DigitalInput(Constants.INTAKE_SENSOR_PORT);

  public IntakeIOSparkMax() {
    sparkMax.restoreFactoryDefaults();
    sparkMax.setInverted(false);
    sparkMax.setIdleMode(IdleMode.kBrake);
    final double GEAR_RATIO = 0.5;
    encoder.setVelocityConversionFactor(GEAR_RATIO);
    encoder.setPositionConversionFactor(GEAR_RATIO);
    pid.setP(.9);
    pid.setI(.002);
    pid.setIZone(1.0);
    // get shuffleboard tab intake
    ShuffleboardTab tab = Shuffleboard.getTab("intake");
    tab.addBoolean("side sensor", () -> this.getSideSensor());
    tab.addBoolean("high sensor", () -> this.getIntakeSensor());
    tab.addDouble("applied otuput", () -> sparkMax.getAppliedOutput());
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.sideSensor = sideSensor.get();
    inputs.intakeSensor = intakeSensor.get();
    inputs.intakeVoltage = sparkMax.getAppliedOutput();
    inputs.intakeStatorCurrent = sparkMax.getOutputCurrent();
    inputs.intakeVelocity = encoder.getVelocity();
    inputs.intakePosition = encoder.getPosition();
  }

  @Override
  public void set(double speed) {
    sparkMax.set(speed);
  }

  @Override
  public void stopMotor() {
    sparkMax.stopMotor();
  }

  @Override
  public boolean getIntakeSensor() {
    return intakeSensor.get();
  }

  @Override
  public boolean getSideSensor() {
    return sideSensor.get();
  }

  @Override
  public boolean getShooterSensor() {
    return shooterSensor.get();
  }

  @Override
  public double getPower() {
    return sparkMax.get();
  }

  @Override
  public double getOutputCurrent() {
    return sparkMax.getOutputCurrent();
  }
  
  @Override
  public double getEncoderValue() {
    return sparkMax.getEncoder().getPosition();
  }

  @Override
  public void setEncoderValue(double encoderPosition) {
    encoder.setPosition(encoderPosition);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public void moveEncoder(double setpoint) {
    pid.setReference(setpoint, ControlType.kPosition);
  }
}