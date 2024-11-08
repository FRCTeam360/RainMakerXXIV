// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpIntake;
import frc.robot.utils.CommandLogger;

public class PowerAmpIntake extends Command {
  private final AmpIntake ampIntake;
  private XboxController opCont = new XboxController(1);

  /** Creates a new PowerArmIntake. */
  public PowerAmpIntake(AmpIntake ampIntake) {
    this.ampIntake = ampIntake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ampIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(opCont.getRightStickButton()) {
      ampIntake.runIntake(0.25);
    } else {

    ampIntake.runIntake(1.0);
    }
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
