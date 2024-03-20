// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;;

public class SnapDrivebaseToAmpAngle extends Command {
  private CommandSwerveDrivetrain swerveDrivetrain;
  /** Creates a new SnapToAmpAngle. */
  public SnapDrivebaseToAmpAngle(CommandSwerveDrivetrain swerveDrivetrain) {
    this.swerveDrivetrain = swerveDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angleToAmp = 0.0;
    if(DriverStation.getAlliance().get() == Alliance.Blue) {
      if(swerveDrivetrain.getAngle() >= 30 && swerveDrivetrain.getAngle() < 150) {
        angleToAmp = 90;
      }
      else if (swerveDrivetrain.getAngle() >= 170 && swerveDrivetrain.getAngle() < 240) {
        angleToAmp = 180;
      }
      swerveDrivetrain.driveFieldCentricFacingAngle(0.0, 0.0, angleToAmp);
    }
    else if (DriverStation.getAlliance().get() == Alliance.Red) {
      if(swerveDrivetrain.getAngle() >= -30 && swerveDrivetrain.getAngle() < -150) {
        angleToAmp = 90;
      }
      if(swerveDrivetrain.getAngle() >= -170 && swerveDrivetrain.getAngle() < -240) {
        angleToAmp = 180;
      }
      swerveDrivetrain.driveFieldCentricFacingAngle(0.0, 0.0, angleToAmp);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
