// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterLinkage;

public class RunShooterLinkage extends Command {
  private final ShooterLinkage shooterLinkage = ShooterLinkage.getInstance();
  private XboxController operatorCont = new XboxController(1);

  /** Creates a new RunShooterLinkage. */
  public RunShooterLinkage() {
    addRequirements(shooterLinkage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(operatorCont.getRightTriggerAxis()) > 0.1) {
      shooterLinkage.run(1.0);
    } else if (Math.abs(operatorCont.getLeftTriggerAxis()) < -0.1) {
      shooterLinkage.run(-1.0);
    } else {
      shooterLinkage.stop();
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
