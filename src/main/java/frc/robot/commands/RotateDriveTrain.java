// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RotateDriveTrain extends Command {
  private Rotation2d desiredAngle = Rotation2d.fromDegrees(90);
  private double veloictyX = 0;
  private double velocityY = 0;
  private CommandSwerveDrivetrain driveTrain;
  //private Constants MaxSpeed; 
  SwerveRequest.FieldCentricFacingAngle drive = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(Constants.MaxSpeed * 0.1).withRotationalDeadband(Constants.MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
  /** Creates a new RotateDriveTrain. */
  public RotateDriveTrain(Rotation2d angle, CommandSwerveDrivetrain drive) {
    driveTrain = drive;
    desiredAngle = angle;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.applyRequest(
      () -> drive.withVelocityX(velocityX).withVelocityY(velocityY) // drive forward with negative y
    .withRotationalRate(desiredAngle)); //drive left with negative x
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
