// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private final Shooter shooter = Shooter.getInstance();
  private XboxController operaterCont = new XboxController(1);

  /** Creates a new RunIntake. */
  public RunShooter() {
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(operaterCont.getLeftY()) > 0.1) {
    shooter.runLeft(1.0);
    } else if (Math.abs(operaterCont.getLeftY()) < -0.1) {
    shooter.runLeft(-1.0);
    } else {
      shooter.runLeft(0.0);
    }
    if (Math.abs(operaterCont.getRightY()) > 0.1) {
      shooter.runRight(1.0);
    }else if (Math.abs(operaterCont.getRightY()) < -0.1) {
      shooter.runRight(-1.0);
    } else {
      shooter.stop(); //u could use ur stop method here
    }
    }

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}