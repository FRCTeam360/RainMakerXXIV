// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveFieldCentricFacingAngle extends Command {
  private final CommandSwerveDrivetrain drivetrain; 
  /** Creates a new FieldCentricFacingAngle. */
  public DriveFieldCentricFacingAngle(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain; 
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.getAlliance().get() == Alliance.Red){
      drivetrain.driveFieldCentricFacingAngle(0, 0, 0, 225.0);
    }else{
      drivetrain.driveFieldCentricFacingAngle(0, 0, 0, 315.0);
    }
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
