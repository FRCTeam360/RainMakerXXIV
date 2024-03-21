// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.PowerFlywheel;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Linkage;
import frc.robot.subsystems.Vision;

public class EndWhenShooterReady extends Command {
  private boolean stop;
  private final Linkage linkage;
  private final Flywheel flywheel;
  private final CommandSwerveDrivetrain swerveDrivetrain;
  // Vision subsystem
  private final Vision vision;
  private Timer timer = new Timer();
  /** Creates a new endWhenShooterRedy. */
  public EndWhenShooterReady(Linkage linkage, Flywheel flywheel, CommandSwerveDrivetrain swerveDrivetrain, Vision vision) {
    this.linkage = linkage;
    this.flywheel = flywheel;
    this.swerveDrivetrain = swerveDrivetrain;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stop = false;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isLinkageAtSetpoint = linkage.isAtSetpoint();
    boolean isFlywheelAboveSetpoint = flywheel.isAboveSetpoint();
    boolean isDrivetrainOnTarget = swerveDrivetrain.isFacingAngle();
    Logger.recordOutput("EndWhenShooterReady: isLinkageAtSetpoint", isLinkageAtSetpoint);
    Logger.recordOutput("EndWhenShooterReady: isFlywheelAboveSetpoint", isFlywheelAboveSetpoint);
    Logger.recordOutput("EndWhenShooterReady: isDrivetrainOnTarget", isDrivetrainOnTarget);
    if ((isLinkageAtSetpoint || timer.get() > 0.3) && ((isFlywheelAboveSetpoint && isDrivetrainOnTarget) || timer.get() > 0.5)) {
      stop = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
