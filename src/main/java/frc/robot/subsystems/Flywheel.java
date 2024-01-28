// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Flywheel extends SubsystemBase {

  private static Flywheel instance;

  private final CANSparkFlex topMotor = new CANSparkFlex(Constants.SHOOTER_TOP_ID, MotorType.kBrushless);
  private final RelativeEncoder topEncoder = topMotor.getEncoder();
  public final SparkPIDController topPIDController = topMotor.getPIDController();

  private final CANSparkFlex bottomMotor = new CANSparkFlex(Constants.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
  private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
  public final SparkPIDController bottomPIDController = bottomMotor.getPIDController();

  private double kP = 0.0055;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kFF = 0.000152;

  private double rpmSetpoint = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel() {
    topMotor.restoreFactoryDefaults();
    topMotor.setInverted(true);
    topMotor.setIdleMode(IdleMode.kBrake);

    bottomMotor.restoreFactoryDefaults();
    bottomMotor.setInverted(false);
    bottomMotor.setIdleMode(IdleMode.kBrake);

    topPIDController.setP(kP);
    topPIDController.setFF(kFF);
    topPIDController.setI(kI);
    topPIDController.setD(kD);

    bottomPIDController.setP(kP);
    bottomPIDController.setFF(kFF);
    bottomPIDController.setI(kI);
    bottomPIDController.setD(kD);

    SmartDashboard.putNumber("Top Velocity", topEncoder.getVelocity());
  }

  public static Flywheel getInstance() {
    if (instance == null) {
      instance = new Flywheel();
    }
    return instance;
  }

  public void runTop(double speed) {
    topMotor.set(speed);
  }

  public void runBottom(double speed) {
    bottomMotor.set(speed);
  }

  public void runBoth(double speed) {
    topMotor.set(speed);
    bottomMotor.set(speed);
  }

  public void setTopRPM(double rpm) {
    topPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void setBottomRPM(double rpm) {
    bottomPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void setBothRPM(double rpm) {
    topPIDController.setReference(rpm, ControlType.kVelocity);
    bottomPIDController.setReference(rpm, ControlType.kVelocity);
  }

  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public double getTopSpeed() {
    return topMotor.get();
  }

  public double getBottomSpeed() {
    return bottomMotor.get();
  }

  public double getTopVelocity() {
    return topEncoder.getVelocity();
  }

  public double getBottomVelocity() {
    return bottomEncoder.getVelocity();
  }

  public boolean isAtSetpoint() {
    return Math.abs(this.getTopSpeed() - rpmSetpoint) < 20.0;
  }

  public boolean isAboveSetpoint() {
    return this.getTopSpeed() >= rpmSetpoint;
  }

  public boolean isBelowSetpoint() {
    return this.getTopSpeed() <= rpmSetpoint - 200.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Velocity", bottomEncoder.getVelocity());

  }
}
