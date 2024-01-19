// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.ShooterLinkageIO;

public class ShooterLinkageIOSparkMax implements ShooterLinkageIO {
  /** Creates a new IntakeIOSparkMax. */
    private final CANSparkMax sparkMax = new CANSparkMax (2, MotorType.kBrushless);
    private final RelativeEncoder encoder = sparkMax.getEncoder();
    private final SparkPIDController pid = sparkMax.getPIDController();

  public ShooterLinkageIOSparkMax() {
    final double GEAR_RATIO = 6.0;
    sparkMax.restoreFactoryDefaults();
    sparkMax.setInverted(false);
    sparkMax.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(360.0/36.0); //360deg / 36:1 gear ratio
    encoder.setPosition(43.0);

    sparkMax.setSoftLimit(SoftLimitDirection.kForward,180f);
    sparkMax.setSoftLimit(SoftLimitDirection.kReverse, 50f);
    sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);

    sparkMax.setClosedLoopRampRate(1.0);
  }
  @Override 
  public void updateInputs(ShooterLinkageIOInputs inputs) {
    inputs.shooterLinkageAngle = encoder.getPosition(); 
  }

  public void set(double speed) {
    sparkMax.set(speed);
  }

  public void stopMotor() {
    sparkMax.stopMotor(); 
  }

  public double getPosition() {
    return encoder.getPosition();
  }
  
  public double get() {
    return sparkMax.get();
  }
  
  public void setPosition(int angle) {
    encoder.setPosition(angle);
  }
  
  public void setFF(double ff) {
    pid.setFF(ff);
  }

  public double getAppliedOutput() {
    return sparkMax.getAppliedOutput();
  }
}

