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

public class TrapSetUpTheSequel extends Command {
  private final Linkage linkage;
  private final AmpArm arm;
  private boolean stop = false;
  private final CommandSwerveDrivetrain drive;
  private final Climber climber;
  private Translation2d initialTranslation;

  /** Creates a new TrapSetUpTheSequel. */
  public TrapSetUpTheSequel(Linkage linkage, AmpArm arm, CommandSwerveDrivetrain drive, Climber climber) {
    this.linkage = linkage;
    this.arm = arm;
    this.drive = drive;
    this.climber = climber;
    addRequirements(linkage, arm, drive, climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stop = false;
    initialTranslation = drive.getPose().getTranslation();
        CommandLogger.logCommandStart(this);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Translation2d currentTranslation2d = drive.getPose().getTranslation();
    drive.robotCentricDrive(0.0, .04, 0.0);
    double distance = Math.abs(currentTranslation2d.getDistance(initialTranslation));
    if (distance > Units.inchesToMeters(8.0)) {
      drive.robotCentricDrive(0, 0, 0);
    }
    if (distance > Units.inchesToMeters(1.0) && distance < 0) {
      arm.setArm(25.0, linkage);
    } else if (distance > Units.inchesToMeters(3.0) && distance < Units.inchesToMeters(2.0)) {
      arm.setArm(45.0, linkage);
    } else if (distance > Units.inchesToMeters(4.0)) {
      arm.setArm(90.0, linkage);
    }

    if (arm.getArmPosition() > 10.0) {
      arm.setWrist(0.0);
    }

    if (Math.abs(arm.getArmPosition() - 90
    ) < 2.0 && Math.abs(arm.getWristPosition()) < 2.0
        && distance > Units.inchesToMeters(7.5)) {
      stop = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopBoth();
    CommandLogger.logCommandEnd(this);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
