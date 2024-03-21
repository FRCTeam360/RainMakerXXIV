// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.io;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public double tx;
        public double ty;
        public double tyFudgeFactor;
        public double tv;
        public double pipeline;
    }

    public double getTX();

    public double getTY();

    public double getTV();

    public Pose2d getBotPose();

    public double getPipeline();

    public void setPipeline(int pipeline);

    public void setLEDMode(int mode);

    public void takeSnapshot();
}
