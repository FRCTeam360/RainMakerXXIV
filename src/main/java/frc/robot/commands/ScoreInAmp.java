// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.AmpIntake;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class ScoreInAmp extends Command {
  private final AmpArm ampArm;
  private final AmpIntake ampIntake;
  private final Linkage linkage;
  
  /** Creates a new ScoreInAmp. */
  public ScoreInAmp(AmpArm ampArm, AmpIntake ampIntake, Linkage linkage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ampArm = ampArm;
    this.ampIntake = ampIntake;
    this.linkage = linkage;
    addRequirements(ampArm, ampIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampArm.setArm(108.5, linkage);
    ampArm.setWrist(140.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampIntake.stop();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
