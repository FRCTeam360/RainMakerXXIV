// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class BasicClimb extends Command {
  private final Climber climber;
  private final Timer timer = new Timer();
  private boolean isDone;

  /** Creates a new BasicCommand. */
  public BasicClimb(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setLeftHeight(58);
    climber.setRightHeight(58);

    if (timer.get() > 0.8) {
      climber.setLeftHeight(-30.0);
      climber.setRightHeight(-30.0);
    }

    // if (climber.getLeftPosition() + 30 < 1.0 && climber.getRightPosition() + 30 < 1.0) {
    //   isDone = true;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
