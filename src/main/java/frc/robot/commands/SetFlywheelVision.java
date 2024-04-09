// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Vision;
import frc.robot.utils.CommandLogger;

public class SetFlywheelVision extends Command {
  private final Flywheel flywheel;
  private final Vision vision; 
  private double setpoint; 
  /** Creates a new SetFlywheel. */
  public SetFlywheelVision(Flywheel flywheel, double setpoint) {
    this.flywheel = flywheel; 
    this.setpoint = setpoint; 
    this.vision = null; 
    addRequirements(flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  /** Creates a new SetFlywheel. */
  public SetFlywheelVision(Flywheel flywheel, double setpoint, Vision vision) {
    this.flywheel = flywheel; 
    this.setpoint = setpoint; 
    this.vision = vision; 
    addRequirements(flywheel);
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
    if (vision != null && vision.isTargetInView()) {
      setpoint = vision.getFlywheelSetpoint();
    }
    flywheel.setIndividualRPM(setpoint, 6750);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
