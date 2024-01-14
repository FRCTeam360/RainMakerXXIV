// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterLinkage extends SubsystemBase {
  private final CANSparkMax motor = new CANSparkMax(Constants.SHOOTER_LINKAGE_ID, MotorType.kBrushless);
  private static ShooterLinkage instance;
  /** Creates a new ShooterLinkage. */
  public ShooterLinkage() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static ShooterLinkage getInstance() {
    if(instance == null) {
    instance = new ShooterLinkage();
    }
    return instance;
  }
  public void run(double speed) {
    motor.set(speed);

  }
  public void stop() {
    motor.stopMotor();
  }
}
