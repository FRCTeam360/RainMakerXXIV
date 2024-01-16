// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTable lime = NetworkTableInstance.getDefault().getTable("limelight");
  /** Creates a new Limelight. */
  public Limelight() {}

  public double getTX() {
    return lime.getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return lime.getEntry("ty").getDouble(0);
  }

  public double getTV() {
    return lime.getEntry("tv").getDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
