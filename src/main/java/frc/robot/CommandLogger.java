// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class CommandLogger {
    static void logCommandStart(Command command) {
        Logger.recordOutput("Command Start: " + command.getName());
    }
}
