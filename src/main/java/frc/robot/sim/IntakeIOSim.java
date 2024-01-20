// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;            
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.IntakeIO;

public class IntakeIOSim implements IntakeIO {
  /** Creates a new IntakeIOSim. */
  private DCMotor sim = DCMotor.getNEO(4);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);
  private final Encoder m_encoder = 
    new Encoder(0, 1);
  private final PWMSparkMax m_motor = new PWMSparkMax(0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  private final FlywheelSim flywheel = new FlywheelSim(sim, 2, 2);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(flywheel.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      flywheel.setInputVoltage(appliedVolts);
    }

    flywheel.update(0.02);

    inputs.intakeSpeed = ((CounterBase) flywheel).get();
    inputs.appliedVolts = appliedVolts;
  }

  @Override
  public void set(double speed) {
    closedLoop = false;
    

  }

  @Override
  public void stopMotor() {
    
  }

  @Override
  public double get() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'get'");
  }
}
