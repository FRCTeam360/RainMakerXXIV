// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.FlywheelIO;
import frc.robot.io.IntakeIO.IntakeIOInputs;

public class FlywheelIOSparkFlex implements FlywheelIO {
  /** Creates a new FlywheelIOSparkMax. */
  private final CANSparkFlex topMotor = new CANSparkFlex(Constants.SHOOTER_TOP_ID, MotorType.kBrushless);
    private final RelativeEncoder topEncoder = topMotor.getEncoder();
    private final SparkPIDController topPIDController = topMotor.getPIDController();

    private final CANSparkFlex bottomMotor = new CANSparkFlex(Constants.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
    private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
    private final SparkPIDController bottomPIDController = bottomMotor.getPIDController();

  public FlywheelIOSparkFlex() {
    double kP = 0.00055;
    double kI = 0.0;
    double kD = 0.0;
    double kFF = 0.000152;
  
    
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
  }

  @Override
  public void setTop(double speed) {
    topMotor.set(speed);
  }

  @Override
  public void setBottom(double speed) {
   bottomMotor.set(speed);
  }

  @Override
  public void setTopReference(double rpm, ControlType kvelocity) {
  topPIDController.setReference(rpm, kvelocity);
  }

  @Override
  public void setBottomReference(double rpm, ControlType kvelocity) {
    bottomPIDController.setReference(rpm, kvelocity);
  }

  @Override
  public void stopTopMotor() {
    topMotor.stopMotor();
  }

  @Override
  public void stopBottomMotor() {
    bottomMotor.stopMotor();
  }

  public double getTopPower() {
   return topMotor.get();
  }

  @Override
  public double getBottomPower() {
    return bottomMotor.get();
  }

  @Override
  public double getTopVelocity() {
    return topEncoder.getVelocity();
  }

  @Override
  public double getBottomVelocity() {
    return bottomEncoder.getVelocity();
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.topSpeed = topMotor.get();
    inputs.bottomSpeed = bottomMotor.get();
  }
}