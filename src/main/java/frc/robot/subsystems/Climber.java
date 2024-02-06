// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  private CANSparkMax leftMotor = new CANSparkMax(Constants.CLIMBER_LEFT_ID, MotorType.kBrushless); // change to proper id
  private CANSparkMax rightMotor = new CANSparkMax(Constants.CLIMBER_RIGHT_ID, MotorType.kBrushless); // change to proper id
  
  /** Creates a new Climber. */
  public Climber() {
    rightMotor.restoreFactoryDefaults();
    leftMotor.restoreFactoryDefaults();
    rightMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setInverted(true);
    leftMotor.setInverted(false);

    leftMotor.follow(rightMotor);
  }

  public void run(double speed) {
    rightMotor.set(speed);
  }

  public void stop() {
    rightMotor.set(0);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
