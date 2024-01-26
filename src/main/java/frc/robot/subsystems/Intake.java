// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
   private final CANSparkMax motor = new CANSparkMax(5, MotorType.kBrushless);
  /** Creates a new Intake. */
  private boolean isComp;

  public Intake() {

    motor.restoreFactoryDefaults();
    motor.setInverted(isComp);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(20);
  }

  public void run(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
