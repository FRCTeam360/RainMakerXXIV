// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANSparkMax Leadmotor = new CANSparkMax(3, MotorType.kBrushless); 
  private final CANSparkMax Followmotor = new CANSparkMax(4, MotorType.kBrushless);
  /** Creates a new Shooter. */
  private boolean isComp;

  public Shooter() {

    Leadmotor.restoreFactoryDefaults();
    Leadmotor.setInverted(isComp);
    Leadmotor.setIdleMode(IdleMode.kBrake);
    Leadmotor.setSmartCurrentLimit(20);
  }

  public void run(double speed) {
    Leadmotor.set(speed);
    Leadmotor.setInverted(true);
    //sets followmotor to invert then follows 
    Followmotor.follow(Leadmotor);
  }


  public void stop() {
    Leadmotor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
