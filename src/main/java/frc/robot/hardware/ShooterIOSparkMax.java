// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.hardware;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.ShooterIO;
import frc.robot.Constants;

public class ShooterIOSparkMax implements ShooterIO {
  /** Creates a new IntakeIOSparkMax. 
    private final CANSparkMax left = new CANSparkMax (3, MotorType.kBrushless);
    private final CANSparkMax right = new CANSparkMax (4, MotorType.kBrushless);
    private final RelativeEncoder encoder = left.getEncoder();
    private final SparkPIDController pid = left.getPIDController();
    final double GEAR_RATIO = 1.0;

  public ShooterIOSparkMax() {
    left.restoreFactoryDefaults();
    left.setInverted(false);
    left.setIdleMode(IdleMode.kBrake);

    right.restoreFactoryDefaults();
    right.setInverted(true);
    right.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // This method will be called once per scheduler run
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = right.getAppliedOutput() * right.getBusVoltage();
    inputs.currentAmps = new double[] {right.getOutputCurrent(), left.getOutputCurrent()};
    inputs.leftSpeed = left.get();
    inputs.rightSpeed = right.get();
  }

  @Override
  public void setLeft(double speed) {
    right.set(speed);
  }

  @Override
  public void setRight(double speed) {
    left.set(speed);
  }

  @Override
  public void stopLeftMotor() {
    left.stopMotor();
  }

  @Override
  public void stopRightMotor() {
    right.stopMotor();
  }

  @Override
  public double getLeft() {
    return left.get();
  }

  @Override
  public double getRight() {
    return right.get();
  }
}
*/
