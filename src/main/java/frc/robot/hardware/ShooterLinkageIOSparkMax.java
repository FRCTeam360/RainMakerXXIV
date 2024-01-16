// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.ShooterLinkageIO;

public class ShooterLinkageIOSparkMax implements ShooterLinkageIO {
  /** Creates a new IntakeIOSparkMax. */
    private final CANSparkMax SparkMax = new CANSparkMax (2, MotorType.kBrushless);
    private final RelativeEncoder encoder = SparkMax.getEncoder();
    private final SparkPIDController pid = SparkMax.getPIDController();

  public ShooterLinkageIOSparkMax() {
    final double GEAR_RATIO = 6;
  }
  @Override 
  public void updateInputs(ShooterLinkageIOInputs SparkMax) {
    // This method will be called once per scheduler run
  }
}

