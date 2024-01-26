// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;



public class SetpointFlywheel extends Command {

  private final Shooter shooter;
  private double setpointRpm;
  
  /** Creates a new SetpointFlywheel. */
  public SetpointFlywheel(Shooter shooter, double setpointRpm) {
    this.shooter = shooter;
    this.setpointRpm = setpointRpm;
    addRequirements(this.shooter);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(setpointRpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopLeft();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    shooter.setSpeed(0.0);
    return false;
  }
}
