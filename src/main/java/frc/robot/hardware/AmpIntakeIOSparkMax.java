// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.AmpArmIO.AmpArmIOInputs;
import frc.robot.io.AmpIntakeIO;

public class AmpIntakeIOSparkMax implements AmpIntakeIO {

  private CANSparkMax motor = new CANSparkMax(Constants.AMP_INTAKE_ID, MotorType.kBrushless);
  private RelativeEncoder encoder = motor.getEncoder();
  /** Creates a new AmpIntakeIOSparkMax. */
  public AmpIntakeIOSparkMax() {}

  @Override
  public void runIntake(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.stopMotor();
  }

  public void updateInputs(AmpIntakeIOInputs inputs) {
    inputs.ampIntakeAmps = motor.getOutputCurrent();
    inputs.ampIntakeVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.ampIntakeVelocity = encoder.getVelocity();
    inputs.ampIntakePosition = encoder.getPosition();
  }


  @Override
  public double getIntakeSpeed() {
    return motor.get();
  }
}