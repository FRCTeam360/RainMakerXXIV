// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Linkage extends SubsystemBase {

  private static Linkage instance;
  private final CANSparkMax motor = new CANSparkMax(Constants.SHOOTER_LINKAGE_ID, MotorType.kBrushless);
  public final RelativeEncoder encoder = motor.getEncoder();
  public final SparkPIDController pidController = motor.getPIDController();
  private double positionSetpoint;

  /** Creates a new ShooterLinkage. */
  public Linkage() {
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(360.0/36.0); //360deg / 36:1 gear ratio
    encoder.setPosition(43.0);

    motor.setSoftLimit(SoftLimitDirection.kForward,180f);
    motor.setSoftLimit(SoftLimitDirection.kReverse, 50f);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    

    motor.setClosedLoopRampRate(1.0);
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

  public void setAngle(double setPoint){
    positionSetpoint = setPoint;
    pidController.setReference(setPoint, CANSparkBase.ControlType.kPosition);
  }

  public double getSpeed() {
    return motor.get();
  }

  public void zero() {
    encoder.setPosition(0);
  }

  public void setFFWScaling(double ff) {
    pidController.setFF(ff * Math.cos(getAngle()));
  }

  public boolean isAtSetpoint() {
    return Math.abs(getAngle() - positionSetpoint) < 1.0;
    }


  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Linkage Angle", getAngle());
    SmartDashboard.putNumber("linkage voltage", motor.getAppliedOutput());
  }


}
