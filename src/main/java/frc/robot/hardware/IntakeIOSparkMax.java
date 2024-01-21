// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.IntakeIO;

public class IntakeIOSparkMax implements IntakeIO {
  /** Creates a new IntakeIOSparkMax. */
    private final CANSparkMax sparkMax = new CANSparkMax(5, MotorType.kBrushless);
    private final RelativeEncoder encoder = sparkMax.getEncoder();
    private final SparkPIDController pid = sparkMax.getPIDController();
    // new hardware class for sensor?
    private final DigitalInput sensor = new DigitalInput(0); // update port later idk what it is

  public IntakeIOSparkMax() {
    sparkMax.restoreFactoryDefaults();
    sparkMax.setInverted(false);
    sparkMax.setIdleMode(IdleMode.kBrake);
    final double GEAR_RATIO = 2.0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeSpeed = sparkMax.get();
    inputs.output = sparkMax.getAppliedOutput();
    inputs.amps = sparkMax.getOutputCurrent();
    inputs.sensor = sensor.get();
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
  public double get() {
    return sparkMax.get();
  }

  @Override
  public double getOutputCurrent() {
    return sparkMax.getOutputCurrent();
  }

  @Override
  public boolean getSensor() {
    return sensor.get();
  }
}
