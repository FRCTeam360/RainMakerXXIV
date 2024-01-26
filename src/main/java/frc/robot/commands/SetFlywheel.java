// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class SetFlywheel extends Command {
  private final Flywheel flywheel = Flywheel.getInstance();
  private final double speed;

  /** Creates a new SetFlywheel. */
  public SetFlywheel(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
    this.speed = speed;}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    flywheel.runBoth(speed *0.8, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stopBoth();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
