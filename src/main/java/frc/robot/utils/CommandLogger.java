// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class CommandLogger {
    // Log the command that is starting
    public static void logCommandStart(Command command) {
        Logger.recordOutput("Command Running: " + command.getClass().getName(), true);
        System.out.println("Print Command Start: " + command.getClass().getName());
    }

    // Log the command that is running
    public static void logCommandRunning(Command command) {
        Logger.recordOutput("Command Running: " + command.getClass().getName(), true);
    }

    // Log the command that is ending
    public static void logCommandEnd(Command command) {
        Logger.recordOutput("Command Running: " + command.getClass().getName(), false);
        System.out.println("Print Command End: " + command.getClass().getName());
    }
}
