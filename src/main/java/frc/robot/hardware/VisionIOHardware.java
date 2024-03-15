// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import frc.robot.io.VisionIO;
import frc.robot.subsystems.Vision;

/** Add your docs here. */
public class VisionIOHardware implements VisionIO{
  Vision vision = new Vision(this);
    public void updateInputs(VisionIOInputs inputs) {
        inputs.tv = vision.getTV() == 1 ? true : false;
        inputs.tx = vision.getTX();
        inputs.ty = vision.getTY();
      }
}
