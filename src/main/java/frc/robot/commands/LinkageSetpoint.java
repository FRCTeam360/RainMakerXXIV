// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Linkage;

public class LinkageSetpoint extends Command {
  private final Linkage linkage;
  private final AmpArm arm;
  /** Creates a new LinkageSetpoint. */
  public LinkageSetpoint(Linkage linkage, AmpArm arm) {
    this.linkage = linkage;
    this.arm = arm;
    addRequirements(linkage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Setpoint", 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Linkage angle", linkage.getAngle());
    double setpoint = SmartDashboard.getNumber("Setpoint", 0.0);
    linkage.setAngle(setpoint, arm);
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
