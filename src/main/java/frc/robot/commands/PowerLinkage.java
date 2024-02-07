// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Linkage;

public class PowerLinkage extends Command {

  private final Linkage shooterLinkage;
  private final XboxController operatorCont = new XboxController(1);

  /** Creates a new RunShooterLinkage. */
  public PowerLinkage(Linkage shooterLinkage) {
    this.shooterLinkage = shooterLinkage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterLinkage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 
    shooterLinkage.run(operatorCont.getLeftY() * 0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterLinkage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
