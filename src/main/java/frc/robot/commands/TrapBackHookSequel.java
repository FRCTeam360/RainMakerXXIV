// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class TrapBackHookSequel extends Command {
  private final Climber climber;
  private final CommandSwerveDrivetrain driveTrain;
  private final AmpArm ampArm;
  private final Linkage linkage;
  private Translation2d intialTranslation2d;

  private double climbUp = 50.0;
  private double climbDown = -57.0;

  private boolean isDone;

  /** Creates a new TrapBackHookSequel. */
  public TrapBackHookSequel(Climber climber, CommandSwerveDrivetrain driveTrain, AmpArm ampArm, Linkage linkage) {
    this.climber = climber;
    this.driveTrain = driveTrain;
    this.ampArm = ampArm;
    this.linkage = linkage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber, driveTrain, ampArm, linkage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
    intialTranslation2d = driveTrain.getPose().getTranslation();
    driveTrain.robotCentricDrive(0, 0, 0);
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setLeftHeight(climbDown, 1);
    climber.setRightHeight(climbDown, 1);
    ampArm.setArm(80.0, linkage);
    ampArm.setWrist(80.0);

    if (Math.abs(climber.getLeftPosition() - climbDown) < 1.0
        && Math.abs(climber.getRightPosition() - climbDown) < 1.0) {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override

  public void end(boolean interrupted) {
    climber.setLeftHeight(climbDown, 1);
    climber.setRightHeight(climbDown, 1);
    CommandLogger.logCommandEnd(this);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
