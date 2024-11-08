// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Linkage;

public class AmpArmGoToZero extends Command {
private final AmpArm ampArm;
  private final Linkage linkage;
  private boolean isDone;  
  /** Creates a new AmpArmGoToZero. */
  public AmpArmGoToZero(AmpArm ampArm, Linkage linkage) {
    this.ampArm = ampArm;
    this.linkage = linkage;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ampArm, linkage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampArm.setArm(0, linkage);
    ampArm.setWrist(0);
    
    if (Math.abs(ampArm.getArmPosition()) < 1.0 && Math.abs(ampArm.getWristPosition()) < 1.0) {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
