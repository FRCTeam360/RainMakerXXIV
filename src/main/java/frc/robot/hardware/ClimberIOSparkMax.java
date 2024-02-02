// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.ClimberIO;

public class ClimberIOSparkMax implements ClimberIO {
   private CANSparkMax leftMotor = new CANSparkMax(325, MotorType.kBrushless); // change to proper id
  private CANSparkMax rightMotor = new CANSparkMax(326, MotorType.kBrushless); // change to proper id

  /** Creates a new ClimberIOSparkMax. */
  public ClimberIOSparkMax() {
    rightMotor.restoreFactoryDefaults();
    leftMotor.restoreFactoryDefaults();
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);

    leftMotor.follow(rightMotor);
  }

  @Override
  public void setLeft(double speed) {
    leftMotor.set(speed);
  }

  @Override
  public void setRight(double speed) {
    rightMotor.set(speed);
  }
}
