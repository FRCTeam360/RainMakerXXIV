// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SnapDrivebaseToAngle extends Command {
   private CommandSwerveDrivetrain swerveDrivetrain;
  
  /** Creates a new SnapDrivebaseToAngle. */
  public SnapDrivebaseToAngle(CommandSwerveDrivetrain swerveDrivetrain) {
    this.swerveDrivetrain = swerveDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double angleToFace = 0.0;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      if (swerveDrivetrain.getAngle() >= 60 && swerveDrivetrain.getAngle() < 180) {
        angleToFace = 120.0;
      }
      else if (swerveDrivetrain.getAngle() >= 180 && swerveDrivetrain.getAngle() < 300) {
        angleToFace = 240.0;
      }
      else if(swerveDrivetrain.getAngle() >= 60 && swerveDrivetrain.getAngle() < 300) {
        angleToFace = 0.0;
      }

      swerveDrivetrain.driveFieldCentricFacingAngle(0.0, 0.0, angleToFace);

   } else if( DriverStation.getAlliance().get() == Alliance.Blue) {
      if (swerveDrivetrain.getAngle() >= -60 && swerveDrivetrain.getAngle() < -180) {
        angleToFace = -120;
      }
      else if (swerveDrivetrain.getAngle() >= -180 && swerveDrivetrain.getAngle() < -300) {
        angleToFace = -240.0;
      }
      else if (swerveDrivetrain.getAngle() >= -300 && swerveDrivetrain.getAngle() < -60
      ) {
        angleToFace = 0.0;
      }
      swerveDrivetrain.driveFieldCentricFacingAngle(0.0, 0.0, angleToFace);

      
   }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
