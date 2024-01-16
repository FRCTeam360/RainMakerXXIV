// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.ShooterLinkageIO;
import frc.robot.io.ShooterLinkageIO.ShooterLinkageIOInputs;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;


public class ShooterLinkage extends SubsystemBase {
  private final ShooterLinkageIO io;
  private final ShooterLinkageIOInputs inputs = new ShooterLinkageIOInputs();

  private final CANSparkMax motor = new CANSparkMax(Constants.SHOOTER_LINKAGE_ID, MotorType.kBrushless);

  /** Creates a new ShooterLinkage. */
  public ShooterLinkage(ShooterLinkageIO io) {
    this.io = io;
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("ShooterLinkage", (LoggableInputs) inputs);
  }

  public void run(double speed) {
    motor.set(speed);
  }
}
