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
   private final DigitalInput sensor1 = new DigitalInput(1);
    private final DigitalInput sensor2 = new DigitalInput(2);
     private final DigitalInput sensor3 = new DigitalInput(3);
      private final DigitalInput sensor4 = new DigitalInput(4);
       private final DigitalInput sensor5 = new DigitalInput(5);
        private final DigitalInput sensor6 = new DigitalInput(6);
         private final DigitalInput sensor7 = new DigitalInput(7);
          private final DigitalInput sensor8 = new DigitalInput(8);
           private final DigitalInput sensor9 = new DigitalInput(9);
            private final DigitalInput sensor10 = new DigitalInput(10);
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
    return sensor2.get();
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
    // SmartDashboard.putBoolean("this sensor sucks 0", getSensor(sensor));
    // SmartDashboard.putBoolean("this sensor sucks 1", getSensor(sensor1));
    SmartDashboard.putBoolean("this sensor sucks 2", getSensor());
    // SmartDashboard.putBoolean("this sensor sucks 3", getSensor(sensor3));
    // SmartDashboard.putBoolean("this sensor sucks 4", getSensor(sensor4));
    // SmartDashboard.putBoolean("this sensor sucks 5", getSensor(sensor5));
    // SmartDashboard.putBoolean("this sensor sucks 6", getSensor(sensor6));
    // SmartDashboard.putBoolean("this sensor sucks 7", getSensor(sensor7));
    // SmartDashboard.putBoolean("this sensor sucks 8", getSensor(sensor8));
    // SmartDashboard.putBoolean("this sensor sucks 9", getSensor(sensor9));
    // SmartDashboard.putBoolean("this sensor sucks 10", getSensor(sensor10));
  }
}
