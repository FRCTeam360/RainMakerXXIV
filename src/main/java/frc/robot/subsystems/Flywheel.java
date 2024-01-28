// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {

  private static Flywheel instance;

  private final CANSparkFlex topMotor = new CANSparkFlex(Constants.SHOOTER_TOP_ID, MotorType.kBrushless);
  private final CANSparkFlex bottomMotor = new CANSparkFlex(Constants.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
  
  public final SparkPIDController topPidController = topMotor.getPIDController();
  public final SparkPIDController bottomPidController = bottomMotor.getPIDController();

  public final RelativeEncoder topEncoder = topMotor.getEncoder();
  public final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();

  private double kP = 0.00055;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kFF = 0.000152;
  
  private double rpmSetpoint = 0.0;

  /** Creates a new Shooter. */
  public Flywheel() {

    topMotor.restoreFactoryDefaults();
    topMotor.setInverted(true);
    topMotor.setIdleMode(IdleMode.kBrake);

    bottomMotor.restoreFactoryDefaults();
    bottomMotor.setInverted(true);
    bottomMotor.setIdleMode(IdleMode.kBrake);
    bottomMotor.follow(topMotor, true);

    topPidController.setP(kP);
    topPidController.setI(kI);
    topPidController.setD(kD);
    topPidController.setFF(kFF);

  }

  public static Flywheel getInstance() {
    if (instance == null) {
      instance = new Flywheel();
    }
    return instance;
  }

  public void run(double speed) {
    topMotor.set(speed);
    //bottomMotor.set(-speed); uneeded bc of lead/follow systm
  }

  public void stop() {
    topMotor.stopMotor();
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

  public void setSpeed(double rpm) {
    //print rpm 
    System.out.println("RPM: " + rpm);
    topMotor.getPIDController().setReference(rpm, CANSparkBase.ControlType.kVelocity);
    rpmSetpoint = rpm;
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

    SmartDashboard.putNumber("Top Speed", getTopSpeed());
    SmartDashboard.putNumber("Bottom Speed", getBottomSpeed());

  }
}
