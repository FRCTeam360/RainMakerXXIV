// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.LinkageIO;

public class LinkageIOSparkMax implements LinkageIO {
  /** Creates a new IntakeIOSparkMax. */
    private final CANSparkMax sparkMax = new CANSparkMax (Constants.WoodBotConstants.WOOD_BOT_LINKAGE_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = sparkMax.getEncoder();
    private final SparkPIDController pidController = sparkMax.getPIDController();

  public LinkageIOSparkMax() {
    final double GEAR_RATIO = 6.0;
    final double kP = 0.01;
    final double kD = 0.0;
    final double kI = 0.0;
    final double kFF = 0.0;

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

    pidController.setP(kP);
    pidController.setD(kD);
    pidController.setI(kI);
    pidController.setFF(kFF);

  }

  @Override 
  public void updateInputs(LinkageIOInputs inputs) {
    inputs.linkageAngle = encoder.getPosition(); 
    inputs.linkageVoltage = sparkMax.getBusVoltage();
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
  
  public void setFF(double ff) {
    pidController.setFF(ff);
  }

  public double getAppliedOutput() {
    return sparkMax.getAppliedOutput();
  }
  
  public void setPosition(double angle) {
    encoder.setPosition(angle);
  }
  public void enableBrakeMode(){
    sparkMax.setIdleMode(IdleMode.kBrake);
  }
  public void disableBrakeMode(){
    sparkMax.setIdleMode(IdleMode.kCoast);
  }
  public boolean isBrakeMode(){
    return sparkMax.getIdleMode() == IdleMode.kBrake;
  }

  public void setReference( double setPoint) {
    pidController.setReference(setPoint, ControlType.kPosition);
  }

  public boolean getZeroButton(){
    return true;
  }
  public boolean getBrakeButton(){
    return true;
  }
}