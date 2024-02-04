// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FieldOrientedDrive extends Command {
  private final XboxController driverController = new XboxController(0);

  private final CommandSwerveDrivetrain driveTrain = TunerConstants.DriveTrain;
  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MAX_SPEED_MPS * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  /** Creates a new TunerXDrive. */
  public FieldOrientedDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveTrain.fieldCentricDrive(driverController.getLeftX(), driverController.getLeftY(),
        driverController.getRightX());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
