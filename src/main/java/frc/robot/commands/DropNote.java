// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.utils.CommandLogger;

public class DropNote extends Command {
  private Intake intake;
  private Flywheel flywheel;

  
  /** Creates a new DropNote. */
  public DropNote(Intake intake, Flywheel flywheel) {
    this.intake = intake;
    this.flywheel = flywheel;
    addRequirements(intake, flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.run(1.0);
    flywheel.setBothRPM(2000.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    flywheel.stop();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
