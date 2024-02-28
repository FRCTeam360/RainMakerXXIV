// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class PowerLinkage extends Command {

  private final Linkage shooterLinkage;
  private final AmpArm arm;
  private final XboxController driverCont = new XboxController(0);

  /** Creates a new RunShooterLinkage. */
  public PowerLinkage(Linkage shooterLinkage, AmpArm arm) {
    this.shooterLinkage = shooterLinkage;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterLinkage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterLinkage.run(getWithDeadband(driverCont.getLeftY() * 0.7), arm);
    // if (operatorCont.getRightTriggerAxis() > 0.1) {
    //   shooterLinkage.run(0.2);
    // } else if (operatorCont.getLeftTriggerAxis() > 0.1) {
    //   shooterLinkage.run(-0.2);
    // } else {
    //   shooterLinkage.stop();
    // }
  }
  
  public double getWithDeadband(double input) {
    if (Math.abs(input) < 0.1) {
      input = 0.0;
    }
    return input;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterLinkage.stop();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
