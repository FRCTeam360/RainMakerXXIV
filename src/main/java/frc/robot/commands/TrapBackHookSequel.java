// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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

  private double climbDown = -57.0;

  private boolean isDone;

  private XboxController operatorCont = new XboxController(Constants.OPERATOR_CONTROLLER);

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

    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setLeftHeight(climbDown, 1);
    climber.setRightHeight(climbDown, 1);

    if (!isDone) {
      ampArm.setArm(90, linkage);
      ampArm.setWrist(90.0);
    }

    if (Math.abs(climber.getLeftPosition() - climbDown) < 1.0
        && Math.abs(climber.getRightPosition() - climbDown) < 1.0) {
      isDone = true;
    }

    if (isDone) {
      ampArm.runArm(getWithDeadband(operatorCont.getLeftY()) * -0.5, linkage);
      ampArm.runWrist(getWithDeadband(operatorCont.getRightY()));

    }

  }

  public double getWithDeadband(double input) {
    if (Math.abs(input) < 0.1) {
      input = 0.0;
    }
    return input;
  }

  // Called once the command ends or is interrupted.
  @Override

  public void end(boolean interrupted) {
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
