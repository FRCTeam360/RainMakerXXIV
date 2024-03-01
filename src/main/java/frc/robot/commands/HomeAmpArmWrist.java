// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Linkage;

public class HomeAmpArmWrist extends Command {
  private final AmpArm ampArm;
  private final Linkage linkage;
  private boolean isDone;
  /** Creates a new HomeAmpArmWrist. */
  public HomeAmpArmWrist(AmpArm arm, Linkage linkage) {
    this.ampArm = arm;
    this.linkage = linkage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm, linkage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    linkage.setAngle(0, ampArm);
    if (linkage.getAngle() < 1.0) {
      
      ampArm.setArm(-78, linkage);
      ampArm.setWrist(70.0);
        
    }
      

    if (ampArm.getArmPosition() + 78 < 1.0 && ampArm.getWristPosition() - 70.0 < 1.0) {
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
