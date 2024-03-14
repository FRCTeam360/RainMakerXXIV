// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utils.UtilMethods;

public class RefineAngle extends Command {
  private final Vision vision;
  private final CommandSwerveDrivetrain drive;
  private boolean stop;
  /** Creates a new RefineAngle. */
  public RefineAngle(Vision vision, CommandSwerveDrivetrain drive) {
    this.vision = vision;
    this.drive = drive;
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
    if(vision.getTV() == 1.0) {
    if(vision.isOnTargetTX()) {
     drive.robotCentricDrive(0,0,0);
    }
    if(vision.getTX() > 0) {
      drive.robotCentricDrive(0,0,-.02);
    } else if(vision.getTX() < 0) {
      drive.robotCentricDrive(0,0,.02);
    }
  } else {
    drive.robotCentricDrive(0,0,0);
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     drive.robotCentricDrive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
