// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private static Shooter instance;
  private final CANSparkMax lead = new CANSparkMax(Constants.SHOOTER_LEAD_ID, MotorType.kBrushless);
  private final CANSparkMax follow = new CANSparkMax(Constants.SHOOTER_FOLLOW_ID, MotorType.kBrushless);

  /** Creates a new Shooter. */
  public Shooter() {

    lead.restoreFactoryDefaults();
    lead.setInverted(false);
    lead.setIdleMode(IdleMode.kBrake);

    follow.restoreFactoryDefaults();
    follow.setInverted(true);
    follow.setIdleMode(IdleMode.kBrake);
    follow.follow(lead);
  }

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }

    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void run(double speed) {
    lead.set(speed);
  }
}
