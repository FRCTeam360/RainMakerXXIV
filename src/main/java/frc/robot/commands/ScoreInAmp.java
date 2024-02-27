// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.AmpIntake;

public class ScoreInAmp extends Command {
  private final AmpArm ampArm;
  private final AmpIntake ampIntake;
  
  /** Creates a new ScoreInAmp. */
  public ScoreInAmp(AmpArm ampArm, AmpIntake ampIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ampArm = ampArm;
    this.ampIntake = ampIntake;
    addRequirements(ampArm, ampIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampArm.setArm(93.3);
    ampArm.setWrist(113.0);

    if (ampArm.getArmPosition() >= 92.0 && ampArm.getWristPosition() >= 111.0) {
      ampIntake.runIntake(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampIntake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
