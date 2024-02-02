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
  private final SparkPIDController topPIDController = topMotor.getPIDController();

  private final CANSparkFlex bottomMotor = new CANSparkFlex(Constants.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
  private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
  private final SparkPIDController bottomPIDController = bottomMotor.getPIDController();

  private double topP = 0.00055;
  private double topI = 0.0;
  private double topD = 0.0;
  private double topFF = 0.000158;

  private double bottomP = 0.00055;
  private double bottomI = 0.0;
  private double bottomD = 0.0;
  private double bottomFF = 0.000155;

  private double rpmSetpoint = 0.0;

  /** Creates a new Flywheel. */
  public Flywheel() {
    topMotor.restoreFactoryDefaults();
    topMotor.setInverted(true);
    topMotor.setIdleMode(IdleMode.kBrake);

    bottomMotor.restoreFactoryDefaults();
    bottomMotor.setInverted(false);
    bottomMotor.setIdleMode(IdleMode.kBrake);

    topPIDController.setP(topP);
    topPIDController.setFF(topFF);
    topPIDController.setI(topI);
    topPIDController.setD(topD);

    bottomPIDController.setP(bottomP);
    bottomPIDController.setFF(bottomFF);
    bottomPIDController.setI(bottomI);
    bottomPIDController.setD(bottomD);

    SmartDashboard.putNumber("Top Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Velocity", bottomEncoder.getVelocity());
    SmartDashboard.putNumber("Top - Bottom Error", 0.0);

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

  public double getTopPower() {
    return topMotor.get();
  }

  public double getBottomPower() {
    return bottomMotor.get();
  }

  public double getTopVelocity() {
    return topMotor.getEncoder().getVelocity();
  }

  public double getBottomVelocity() {
    return bottomMotor.getEncoder().getVelocity();
  }

  public boolean isAtSetpoint() {
    return Math.abs(this.getTopVelocity() - rpmSetpoint) < 30.0;
  }

  public boolean isAboveSetpoint() {
    return this.getTopVelocity() >= rpmSetpoint;
  }

  public boolean isBelowSetpoint() {
    return this.getTopVelocity() <= rpmSetpoint - 30.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Velocity", topEncoder.getVelocity());
    SmartDashboard.putNumber("Bottom Velocity", bottomEncoder.getVelocity());
    SmartDashboard.putNumber("Top - Bottom Error", topEncoder.getVelocity() - bottomEncoder.getVelocity());
  }
}
