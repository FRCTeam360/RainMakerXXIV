// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Objects;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private final NetworkTable lime = NetworkTableInstance.getDefault().getTable("limelight");
  /** Creates a new Limelight. */
  public Vision() {}

  public void blink(){
    lime.getEntry("ledMode").setNumber(2);
  }
  public void lightsOut(){
    lime.getEntry("ledMode").setNumber(1);
  }

  public double getTX() {
    return lime.getEntry("tx").getDouble(0);
  }

  public double getTY() {
    return lime.getEntry("ty").getDouble(0);
  }

  public double getTV() {
    return lime.getEntry("tv").getDouble(0);
  }

  public double getPipeline() {
    return lime.getEntry("getpipe").getDouble(0);
  }

  public double[] getBotpose() {
    return lime.getEntry("botpose").getDoubleArray(new double[6]);
  }

  public void setPipeline(int pipeline) {
    lime.getEntry("pipeline").setNumber(pipeline);
  }
  
  public double getLinkageSetpoint() {
    return 0.0; // add linkage regression equation thing
  }

  public double getFlywheelSetpoint() {
    return 0.0; // add flywheel regression equation thing
  }

  public boolean isOnTargetTX() {
    if(Math.abs(getTX()) < 3.0) {
      return true;
    }
    return false;
  }
  
  // Returns true if the target is in view
  public boolean isTargetInView() {
    return getTV() == 1;
  }

  @Override
  public void periodic() {
    if (Objects.nonNull(DriverStation.getAlliance())) {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
        setPipeline(0);
      }   else if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        setPipeline(1);
      }
   }
    // This method will be called once per scheduler run
  }
}
