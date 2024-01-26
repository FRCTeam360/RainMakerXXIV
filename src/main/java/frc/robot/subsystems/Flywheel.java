// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {

  private static Flywheel instance;
  // private final CANSparkMax leftMotor = new CANSparkMax(Constants.SHOOTER_LEFT_ID, MotorType.kBrushless);
  // private final CANSparkMax rightMotor = new CANSparkMax(Constants.SHOOTER_RIGHT_ID, MotorType.kBrushless);
  private final CANSparkFlex leftMotor = new CANSparkFlex(Constants.SHOOTER_LEFT_ID, MotorType.kBrushless);
  private final CANSparkFlex rightMotor = new CANSparkFlex(Constants.SHOOTER_RIGHT_ID, MotorType.kBrushless);

  public final SparkPIDController leftPidController = leftMotor.getPIDController();
  public final SparkPIDController rightPidController = rightMotor.getPIDController();
  /** Creates a new Shooter. */
  public Flywheel() {

    leftMotor.restoreFactoryDefaults();
    leftMotor.setInverted(false);
    leftMotor.setIdleMode(IdleMode.kBrake);

    rightMotor.restoreFactoryDefaults();
    rightMotor.setInverted(true);
    rightMotor.setIdleMode(IdleMode.kBrake);
  }

  public static Flywheel getInstance() {
    if (instance == null) {
      instance = new Flywheel();
    }
    return instance;
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

  public void stopLeft() {
    leftMotor.stopMotor();
  }

  public void stopRight() {
    rightMotor.stopMotor();
  }

  public void stopBoth() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public double getLeftSpeed() {
    return leftMotor.get();
  }

  public double getRightSpeed() {
    return rightMotor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Speed", getLeftSpeed());
    SmartDashboard.putNumber("Right Speed", getRightSpeed());
  }
}