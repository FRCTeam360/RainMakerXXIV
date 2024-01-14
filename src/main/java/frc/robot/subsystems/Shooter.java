// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class Shooter extends SubsystemBase {
  private final CANSparkMax left = new CANSparkMax(Constants.SHOOTER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkMax right = new CANSparkMax(Constants.SHOOTER_RIGHT_ID, MotorType.kBrushless);
  private static Shooter instance;


  /** Creates a new Intake. */
  public Shooter() {
    left.restoreFactoryDefaults();
    left.setInverted(true);
    left.setIdleMode(IdleMode.kBrake);

    right.restoreFactoryDefaults();
    right.setInverted(false);
    right.setIdleMode(IdleMode.kBrake);
  }



  public static Shooter getInstance() {
    if(instance == null) {
      instance = new Shooter();
    }
    return instance;

  }

  public void runLeft(double speed) {
    left.set(speed);
  }

  public void runRight(double speed) {
    right.set(speed);

  }
  public void stop(){
    left.stopMotor();
    right.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
