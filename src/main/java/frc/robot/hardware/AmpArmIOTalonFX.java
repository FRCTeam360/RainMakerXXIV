// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.AmpArmIO;

public class AmpArmIOTalonFX implements AmpArmIO {

  private final TalonFX armMotor = new TalonFX(Constants.AMP_ARM_ID, Constants.CANIVORE_NAME);
  private final TalonFX wristMotor = new TalonFX(Constants.AMP_WRIST_ID, Constants.CANIVORE_NAME);

  private final double ARM_RATIO = 1.68; // degrees / motor rotation
  private final double WRIST_RATIO = 11.25; // degrees / motor rotation

  /** Creates a new AmpArmIOTalonFX. */
  public AmpArmIOTalonFX() {
    armMotor.getConfigurator().apply(new TalonFXConfiguration());
    armMotor.setInverted(false);
    armMotor.setNeutralMode(NeutralModeValue.Coast);

    wristMotor.getConfigurator().apply(new TalonFXConfiguration());
    wristMotor.setInverted(false);
    wristMotor.setNeutralMode(NeutralModeValue.Coast);

    TalonFXConfiguration armConfig = new TalonFXConfiguration();
    armConfig.Feedback.withSensorToMechanismRatio(1 /ARM_RATIO);

    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    wristConfig.Feedback.withSensorToMechanismRatio(1 / WRIST_RATIO);

    armMotor.getConfigurator().apply(armConfig);
    wristMotor.getConfigurator().apply(wristConfig);
  }

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

  @Override
  public void zeroWrist() {
    wristMotor.setPosition(0.0);
  }

  @Override
  public void zeroArm() {
    armMotor.setPosition(0.0);
  }
}
