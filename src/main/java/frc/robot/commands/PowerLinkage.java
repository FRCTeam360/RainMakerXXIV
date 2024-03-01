// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class PowerLinkage extends Command {

  private final Linkage linkage;
  private final AmpArm arm;
  private final CommandXboxController operatorController = new CommandXboxController(1);

  /** Creates a new RunShooterLinkage. */
  public PowerLinkage(Linkage linkage, AmpArm arm) {
    this.linkage = linkage;
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(linkage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //linkage.run(getWithDeadband(operatorController.getLeftY() * 0.7), arm);
  
    //for charlie
    while (operatorController.pov(90).getAsBoolean()) {
      linkage.run(0.5, arm);
    } 
    
    while(operatorController.pov(270).getAsBoolean()){
      linkage.run(-0.5, arm);
    }

    CommandLogger.logCommandRunning(this);
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
    linkage.stop();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
