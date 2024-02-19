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
  private final CANSparkFlex leftMotor = new CANSparkFlex(Constants.FLYWHEEL_LEFT_ID, MotorType.kBrushless);
    private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private final SparkPIDController leftPIDController = leftMotor.getPIDController();

    private final CANSparkFlex rightMotor = new CANSparkFlex(Constants.FLYWHEEL_RIGHT_ID, MotorType.kBrushless);
    private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
    private final SparkPIDController rightPIDController = rightMotor.getPIDController();

    private final double VELOCITY_CONVERSION = 36.0/24.0; //24 motor rotations = 36 flywheel rotations (1.5)

  public FlywheelIOSparkFlex() {
    double kP = 0.0006;
    double kI = 0.0;
    double kD = 0.0;
    double kFF = 0.000112;
  
    
    leftMotor.restoreFactoryDefaults();
    leftMotor.setInverted(false);
    leftMotor.setIdleMode(IdleMode.kCoast);

    rightMotor.restoreFactoryDefaults();
    rightMotor.setInverted(true);
    rightMotor.setIdleMode(IdleMode.kCoast);

    leftPIDController.setP(kP);
    leftPIDController.setFF(kFF);
    leftPIDController.setI(kI);
    leftPIDController.setD(kD);

    rightPIDController.setP(kP);
    rightPIDController.setFF(kFF);
    rightPIDController.setI(kI);
    rightPIDController.setD(kD);

    leftEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION);
    rightEncoder.setVelocityConversionFactor(VELOCITY_CONVERSION);
  }

  @Override
  public void setLeft(double speed) {
    leftMotor.set(speed);
  }


  @Override
  public void setRight(double speed) {
   rightMotor.set(speed);
  }

  @Override
  public void setLeftReference(double rpm, ControlType kvelocity) {
  leftPIDController.setReference(rpm, kvelocity);
  }

  @Override
  public void setRightReference(double rpm, ControlType kvelocity) {
    rightPIDController.setReference(rpm, kvelocity);
  }

  @Override
  public void stopLeftMotor() {
    leftMotor.stopMotor();
  }

  @Override
  public void stopRightMotor() {
    rightMotor.stopMotor();
  }

  public double getLeftPower() {
   return leftMotor.get();
  }

  @Override
  public double getRightPower() {
    return rightMotor.get();
  }

  @Override
  public double getLeftVelocity() {
    return leftEncoder.getVelocity();
  }

  @Override
  public double getRightVelocity() {
    return rightEncoder.getVelocity();
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.topSpeed = leftMotor.get();
    inputs.bottomSpeed = rightMotor.get();
  }
}