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
<<<<<<< HEAD
import frc.robot.io.LinkageIO;
import frc.robot.io.LinkageIOInputsAutoLogged;

public class Linkage extends SubsystemBase {
  private final LinkageIO io;
  private final LinkageIOInputsAutoLogged inputs = new LinkageIOInputsAutoLogged();

  /** Creates a new ShooterLinkage. */
  public Linkage(LinkageIO io) {
    this.io = io;
=======
import frc.robot.generated.TunerConstants;

public class Linkage extends SubsystemBase {

  private static Linkage instance;
  private final CANSparkMax motor = new CANSparkMax(Constants.SHOOTER_LINKAGE_ID, MotorType.kBrushless);
  public final RelativeEncoder encoder = motor.getEncoder();
  public final SparkPIDController pidController = motor.getPIDController();
  private double positionSetpoint;

  private static final double STARTING_ANGLE = 50.0;

  static XboxController driverCont = new XboxController(0);

  static CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private double kP = 0.1;
  private double kD = 0.0;
  private double kI = 0.0;
  private double kFF = 0.0; // :(

  /** Creates a new ShooterLinkage. */
  public Linkage() {
    motor.restoreFactoryDefaults();
    motor.setInverted(true);
    motor.setIdleMode(IdleMode.kBrake);

    encoder.setPositionConversionFactor(360.0 / 36.0); // 360deg / 36:1 gear ratio
    encoder.setPosition(STARTING_ANGLE);

    motor.setSoftLimit(SoftLimitDirection.kForward, 180f);
    motor.setSoftLimit(SoftLimitDirection.kReverse, 50f);
    motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    motor.setClosedLoopRampRate(1.0);

    pidController.setP(kP);
    pidController.setD(kD);
    pidController.setI(kI);
    pidController.setFF(kFF);

  }

  public static Linkage getInstance() {
    if (instance == null) {
      instance = new Linkage();
    }

    return instance;
>>>>>>> Woodbot
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

<<<<<<< HEAD
  public void setAngle(int setPoint){
    io.setReference(setPoint, CANSparkBase.ControlType.kPosition);
=======
  public void setAngle(double setPoint) {
    positionSetpoint = setPoint;
    pidController.setReference(setPoint, CANSparkBase.ControlType.kPosition);
>>>>>>> Woodbot
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

  public boolean isAtSetpoint() {
    return Math.abs(getAngle() - positionSetpoint) < 1.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Linkage", inputs);
    SmartDashboard.putNumber("Linkage Angle", getAngle());
<<<<<<< HEAD
    SmartDashboard.putNumber("linkage voltage", io.getAppliedOutput());
  }


}
=======
    SmartDashboard.putNumber("Linkage Voltage", motor.getAppliedOutput());
    SmartDashboard.putNumber("Linkage Error", 85-getAngle());

  }

}
>>>>>>> Woodbot
