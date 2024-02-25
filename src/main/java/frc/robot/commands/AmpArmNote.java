// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpIntake;

public class AmpArmNote extends Command {
  private final AmpIntake intake;
  private boolean stop;
  private enum Cases {
    NO_NOTE,
    NOTE,
    NOTE_CENTERED
  }
  private Cases state = Cases.NO_NOTE;
  /** Creates a new AmpArmMove. */
  public AmpArmNote(AmpIntake intake) {

    this.intake = intake;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case NO_NOTE:
       if(intake.getAmps() > 25) {
        state = Cases.NOTE;
       }
      intake.runIntake(-.2);
      break;
      case NOTE:
       
    }
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
