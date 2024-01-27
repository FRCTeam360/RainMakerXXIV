// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static Intake instance;
  private final DigitalInput sensor = new DigitalInput(0); // update port later idk what it is
  private final CANSparkMax motor = new CANSparkMax(Constants.INTAKE_ID, MotorType.kBrushless);

  /** Creates a new Intake. */
  public Intake() {
    motor.restoreFactoryDefaults();
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kCoast);
  }

  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  public boolean getSensor() {
    return sensor.get();
  }

  public void run(double speed) {
    motor.set(speed);
  }
  
  public void stop() {
    motor.stopMotor();
  }

  public double getAmps() {
    return motor.getOutputCurrent();
  }

  public double getSpeed() {
    return motor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Speed", getSpeed());
    SmartDashboard.putNumber("Intake Amps", getAmps());
    SmartDashboard.putBoolean("this sensor sucks", getSensor());
  }
}
