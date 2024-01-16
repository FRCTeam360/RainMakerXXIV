// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Linkage extends SubsystemBase {

  private static Linkage instance;
  private final CANSparkMax motor = new CANSparkMax(Constants.SHOOTER_LINKAGE_ID, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getEncoder();
  public final SparkPIDController pidController = motor.getPIDController();

  /** Creates a new ShooterLinkage. */
  public Linkage() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(360.0/36.0); //360deg / 36:1 gear ratio

    motor.setSoftLimit(SoftLimitDirection.kForward,-20f);
    motor.setSoftLimit(SoftLimitDirection.kReverse, -174f);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

  }

  public static Linkage getInstance() {
    if (instance == null) {
      instance = new Linkage();
    }

    return instance;
  }
  
  public void run(double speed) {
    motor.set(speed);
  }

  public void stop() {
    motor.stopMotor();
  }

  public double getAngle() {
    return encoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Linkage Angle", getAngle());
  }


}
