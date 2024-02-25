// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.AmpArmIO.AmpArmIOInputs;
import frc.robot.io.AmpIntakeIO;

public class AmpIntakeIOSparkMax implements AmpIntakeIO {

  private CANSparkMax motor = new CANSparkMax(Constants.AMP_INTAKE_ID, MotorType.kBrushless);
  private PIDController pid = motor.getEncoder();

  private final double GEAR_RATIO = 1.0;

  /** Creates a new AmpIntakeIOSparkMax. */
  public AmpIntakeIOSparkMax() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void runIntake(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.stopMotor();
  }

  public double getAmps() {
    return motor.getOutputCurrent();
  }

  public double getEncoderPosition() {
    return motor.getEncoder().getPosition();
  }

  public void moveEncoder() {
    pidController.setReference()
  }

  public void updateInputs(AmpIntakeIOInputs inputs) {
    inputs.intakeSpeed = motor.get();
    inputs.intakeAmps = motor.getAppliedOutput();
  }

  @Override
  public double getIntakeSpeed() {
    return motor.get();
  }
}
