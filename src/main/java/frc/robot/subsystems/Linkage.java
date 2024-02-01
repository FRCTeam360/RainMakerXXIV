// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.LinkageIO;
import frc.robot.io.LinkageIOInputsAutoLogged;

public class Linkage extends SubsystemBase {
  private final LinkageIO io;
  private final LinkageIOInputsAutoLogged inputs = new LinkageIOInputsAutoLogged();

  /** Creates a new ShooterLinkage. */
  public Linkage(LinkageIO io) {
    this.io = io;


public void run(double) {
}
public void setAngle(double linkageSetpoint) {
}
public boolean isAtSetpoint() {
	return false;
}import frc.robot.generated.TunerConstants;

  private static Linkage instance;
  private final CANSparkMax motor = new CANSparkMax(Constants.SHOOTER_LINKAGE_ID, MotorType.kBrushless);
  public final RelativeEncoder encoder = motor.getEncoder();
  public final SparkPIDController pidController = motor.getPIDController();
  private double positionSetpoint;

  private static final double STARTING_ANGLE = 50.0;

  static XboxController driverCont = new XboxController(0);

  static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  }

  public static Linkage getInstance() {
    if (instance == null) {
      instance = new Linkage();
    }

    return instance;
  }

  public void run(double speed) {
    io.set(speed);
  }

  public void stop() {
    io.stopMotor();
  }

  public double getAngle() {
    return io.getPosition();
  }

  public void setAngle(int setPoint){
    io.setReference(setPoint, CANSparkBase.ControlType.kPosition);
    positionSetpoint = setPoint;
    pidController.setReference(setPoint, CANSparkBase.ControlType.kPosition);
  }

  public double getSpeed() {
    return io.get();
  }

  public void zero() {
    io.setPosition(0);
  }

  public void setEncoderTo90() {
    encoder.setPosition(90);
  }

  public void setFFWScaling(double ff) {
    io.setFF(ff * Math.cos(getAngle()));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Linkage", inputs);
    SmartDashboard.putNumber("Linkage Angle", getAngle());
    SmartDashboard.putNumber("linkage voltage", io.getAppliedOutput());
    SmartDashboard.putNumber("Linkage Voltage", motor.getAppliedOutput());
    SmartDashboard.putNumber("Linkage Error", 85-getAngle());

  }

}
