// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private CANSparkMax leftMotor = new CANSparkMax(325, MotorType.kBrushless); // change to proper id
  private CANSparkMax rightMotor = new CANSparkMax(326, MotorType.kBrushless); // change to proper id
  /** Creates a new Climber. */
  public Climber() {
    rightMotor.restoreFactoryDefaults();
    leftMotor.restoreFactoryDefaults();
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);
  }

  public void runLeft(double speed) {
    leftMotor.set(speed);
  }

  public void runRight(double speed) {
    rightMotor.set(speed);
  }

  public void runBoth(double leftSpeed, double rightSpeed) {
    leftMotor.set(leftSpeed);
    rightMotor.set(rightSpeed);
  }

  public void stop() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
