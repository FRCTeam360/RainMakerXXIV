// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Linkage;

public class SetLinkage extends Command {
  private final Linkage linkage = Linkage.getInstance();
  private double linkageAngle;

  /** Creates a new SetLinkage. */
  public SetLinkage(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(linkage);
    linkageAngle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linkage.pidController.setReference(linkageAngle, ControlType.kPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
