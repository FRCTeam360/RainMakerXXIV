// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class SetClimbers extends Command {
  private final Climber climber;
  private final double setPoint;

  private boolean isDone;
  private final boolean shouldFinish;

  /** Creates a new SetClimbers. */
  public SetClimbers(Climber climber, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.setPoint = setPoint;
    this.shouldFinish = false;
    addRequirements(this.climber);
  }

  /** Creates a new SetClimbers. */
  public SetClimbers(Climber climber, double setPoint, boolean shouldFinish) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.setPoint = setPoint;
    this.shouldFinish = shouldFinish;
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setLeftHeight(setPoint, 1); //might need to be changed pre sammamy
    climber.setRightHeight(setPoint, 1);

    if (shouldFinish && Math.abs(climber.getLeftPosition() - setPoint) < 1.0 && Math.abs(climber.getRightPosition() - setPoint) < 1.0) {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
