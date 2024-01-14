// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.IntakeIO;

public class IntakeIOSparkMax implements IntakeIO {
  /** Creates a new IntakeIOSparkMax. */
    private final CANSparkMax leader = new CANSparkMax (0, MotorType.kBrushless);
    private final CANSparkMax follower = new CANSparkMax (1, MotorType.kBrushless);
    private final RelativeEncoder encoder = leader.getEncoder();
    private final SparkPIDController pid = leader.getPIDController();

  public IntakeIOSparkMax() {
    final double GEAR_RATIO = 8.12;
    follower.follow(leader);
  }
  @Override
  public void updateInputs(IntakeIOInputs SparkMax) {
    // This method will be called once per scheduler run
  }
}
