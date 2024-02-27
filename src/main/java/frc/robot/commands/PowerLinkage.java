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
  private final double angle;
  private final XboxController driverCont = new XboxController(0);

  /** Creates a new RunShooterLinkage. */
  public PowerLinkage(Linkage shooterLinkage, double ampAngle) {
    this.shooterLinkage = shooterLinkage;
    this.angle = ampAngle;
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
    if(angle > -74 && angle < 0) {
      shooterLinkage.stop();
    } else {
      shooterLinkage.run(driverCont.getLeftY() * 0.7);
    }
    // if (operatorCont.getRightTriggerAxis() > 0.1) {
    //   shooterLinkage.run(0.2);
    // } else if (operatorCont.getLeftTriggerAxis() > 0.1) {
    //   shooterLinkage.run(-0.2);
    // } else {
    //   shooterLinkage.stop();
    // }
    CommandLogger.logCommandRunning(this);
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
