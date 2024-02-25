// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;

public class PowerAmpArm extends Command {
  private final AmpArm ampArm;
  private final XboxController operatorCont = new XboxController(1);

  /** Creates a new PowerArm. */
  public PowerAmpArm(AmpArm ampArm) {
    this.ampArm = ampArm;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ampArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampArm.avoidWristCollision(0.0);
    ampArm.runArm(-operatorCont.getRightY() * 0.5);
    // ampArm.runWrist(operatorCont.getRightY() * 0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampArm.stopArm();
    ampArm.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
