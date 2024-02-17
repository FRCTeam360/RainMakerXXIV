// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.AmpArmIO;

public class AmpArmIOTalonFX implements AmpArmIO {

  private final TalonFX armMotor = new TalonFX(Constants.AMP_ARM_ID, "Default Name");
  private final TalonFX wristMotor = new TalonFX(Constants.AMP_WRIST_ID, "Default Name");
  
  /** Creates a new AmpArmIOTalonFX. */
  public AmpArmIOTalonFX() {}

  @Override
  public void runArm(double speed) {
    armMotor.set(speed);
  }

  @Override
  public void runWrist(double speed) {
    wristMotor.set(speed);
  }

  public void stopArm() {
    armMotor.stopMotor();
  }

  public void stopWrist() {
    wristMotor.stopMotor();
  }

  public void updateInputs(AmpArmIOInputs inputs) {
    inputs.armSpeed = armMotor.get();
    inputs.wristSpeed = wristMotor.get();
    inputs.armAngle = armMotor.getPosition().getValueAsDouble();
    inputs.wristAngle = wristMotor.getPosition().getValueAsDouble();
    inputs.armAmps = armMotor.getStatorCurrent().getValueAsDouble();
    inputs.wristAmps = wristMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getWristPosition() {
    return wristMotor.getPosition().getValueAsDouble();
  }
}
