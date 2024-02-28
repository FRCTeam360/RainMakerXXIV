// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Linkage;

public class SetLinkage extends Command {
  private final Linkage linkage;
  private final AmpArm arm;
  private double setpoint;
  /** Creates a new SetLinkageTa\
   * lon. */
  public SetLinkage(Linkage linkage, double setpoint, AmpArm arm) {
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    this.linkage = linkage;
    this.arm = arm;
    addRequirements(linkage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //SmartDashboard.putNumber("error", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("error", -7.0 - linkage.getAngle());
    linkage.setAngle(setpoint, arm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
