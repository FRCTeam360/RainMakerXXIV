// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

/** Add your docs here. */
public class UtilMethods {
    public static double squareInput(double input) {
        return Math.signum(input) * Math.pow(input, 2);
    }
}
