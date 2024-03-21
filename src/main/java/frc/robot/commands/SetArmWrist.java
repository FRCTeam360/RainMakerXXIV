// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Linkage;

public class SetArmWrist extends Command {
  private final AmpArm ampArm;
  private final Linkage linkage;
  private final double armAngle;
  private final double wristAngle;

  /** Creates a new SetAmpArm. */
  public SetArmWrist(AmpArm ampArm, Linkage linkage, double armAngle, double wristAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ampArm = ampArm;
    this.linkage = linkage;
    this.armAngle = armAngle;
    this.wristAngle = wristAngle;
    addRequirements(ampArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampArm.setArm(armAngle, linkage);
    ampArm.setWrist(wristAngle);
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
