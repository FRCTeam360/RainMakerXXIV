// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterLinkage;

public class RunShooterLinkage extends Command {
  private ShooterLinkage shooterLinkage;
  private final XboxController operatorCont = new XboxController(1);

  /** Creates a new RunShooterLinkage. */
  public RunShooterLinkage() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterLinkage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (operatorCont.getRightTriggerAxis() > 0.1) {
      shooterLinkage.run(operatorCont.getRightTriggerAxis());
    } else if (operatorCont.getLeftTriggerAxis() > 0.1) {
      shooterLinkage.run(-operatorCont.getLeftTriggerAxis());
    }
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
