// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.PowerFlywheel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Linkage;

public class EndWhenShooterReady extends Command {
  private boolean stop;
  private final Linkage linkage;
  private final Flywheel flywheel;
  private final CommandSwerveDrivetrain swerveDrivetrain;
  /** Creates a new endWhenShooterRedy. */
  public EndWhenShooterReady(Linkage linkage, Flywheel flywheel, CommandSwerveDrivetrain swerveDrivetrain) {
    this.linkage = linkage;
    this.flywheel = flywheel;
    this.swerveDrivetrain = swerveDrivetrain;
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
    if (linkage.isAtSetpoint() && flywheel.isAtSetpoint() && swerveDrivetrain.isFacingAngle()) {
      stop = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
