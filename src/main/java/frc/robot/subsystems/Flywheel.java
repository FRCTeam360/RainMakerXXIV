// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {

  private static Flywheel instance;
  // private final CANSparkMax leftMotor = new CANSparkMax(Constants.SHOOTER_LEFT_ID, MotorType.kBrushless);
  // private final CANSparkMax rightMotor = new CANSparkMax(Constants.SHOOTER_RIGHT_ID, MotorType.kBrushless);
  private final CANSparkFlex topMotor = new CANSparkFlex(Constants.SHOOTER_TOP_ID, MotorType.kBrushless);
  private final CANSparkFlex bottomMotor = new CANSparkFlex(Constants.SHOOTER_BOTTOM_ID, MotorType.kBrushless);
  private double rpmSetpoint = 0.0;

  public final SparkPIDController leftPidController = topMotor.getPIDController();
  public final SparkPIDController rightPidController = bottomMotor.getPIDController();
  

  /** Creates a new Shooter. */
  public Flywheel() {

    topMotor.restoreFactoryDefaults();
    topMotor.setInverted(false);
    topMotor.setIdleMode(IdleMode.kBrake);

    bottomMotor.restoreFactoryDefaults();
    bottomMotor.setInverted(true);
    bottomMotor.setIdleMode(IdleMode.kBrake);
    bottomMotor.follow(topMotor);
  }

  public static Flywheel getInstance() {
    if (instance == null) {
      instance = new Flywheel();
    }
    return instance;
  }

  public void run(double speed) {
    topMotor.set(speed);
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
  public void setSpeed(double rpm){
    leftPidController.setReference(rpm, CANSparkBase.ControlType.kVelocity);
    rpmSetpoint = rpm;
  }

  public boolean isAtSetpoint(){
    return Math.abs(this.getTopSpeed() - rpmSetpoint) < 20.0;
    
  }
  public boolean isAboveSetpoint(){
    return this.getTopSpeed() >= rpmSetpoint;
  }
  public boolean isBelowSetpoint(){
    return this.getTopSpeed() <= rpmSetpoint - 200.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Speed", getTopSpeed());
    SmartDashboard.putNumber("Right Speed", getBottomSpeed());
  }
}
