// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Linkage;

public class LinkageSetpoint extends Command {
  private Linkage linkage;
  /** Creates a new LinkageSetpoint. */
  public LinkageSetpoint(Linkage linkage) {
    this.linkage = linkage;
    addRequirements(linkage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Setpoint", 0.0);
    SmartDashboard.putNumber("Linkage angle", linkage.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double goalSetpoint = SmartDashboard.getNumber("Setpoint", 0.0);
    System.out.println(goalSetpoint);
    linkage.setReference(goalSetpoint);
    SmartDashboard.putNumber("Setpoint", goalSetpoint);
    SmartDashboard.putBoolean("Linkage at setpoint?", linkage.isAtSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    linkage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}