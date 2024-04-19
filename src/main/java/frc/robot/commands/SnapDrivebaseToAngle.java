// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.UtilMethods;

public class SnapDrivebaseToAngle extends Command {
  private final XboxController driverController = new XboxController(0);
  private CommandSwerveDrivetrain swerveDrivetrain;
  double x = 1;
  double angleToFace = 5.0;

  /** Creates a new SnapDrivebaseToAngle. */
  public SnapDrivebaseToAngle(CommandSwerveDrivetrain swerveDrivetrain) {
    this.swerveDrivetrain = swerveDrivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      x = -1;
    } else {
      x = 1;
    }
    // if (DriverStation.getAlliance().get() == Alliance.Red) {

    // if (swerveDrivetrain.getAngle() >= 0 && swerveDrivetrain.getAngle() <= 90) {
    // angleToFace = 60.0;
    // } else if (swerveDrivetrain.getAngle() >= 90 && swerveDrivetrain.getAngle()
    // <= 150) {
    // angleToFace = 120.0;
    // } else if (swerveDrivetrain.getAngle() >= 150 || swerveDrivetrain.getAngle()
    // <= -150.0) {
    // angleToFace = 180.0;
    // } else if (swerveDrivetrain.getAngle() >= -150 && swerveDrivetrain.getAngle()
    // <= -90.0) {
    // angleToFace = -120.0;
    // } else if (swerveDrivetrain.getAngle() >= -90.0 &&
    // swerveDrivetrain.getAngle() <= 0.0) {
    // angleToFace = -60.0;
    // }

    // } else {
    if (swerveDrivetrain.getAngle() >= -30 && swerveDrivetrain.getAngle() <= 30) {
      angleToFace = 0.0;
    } else if (swerveDrivetrain.getAngle() >= 30 && swerveDrivetrain.getAngle() <= 90) {
      angleToFace = 60.0;
    } else if (swerveDrivetrain.getAngle() >= 90 && swerveDrivetrain.getAngle() <= 150.0) {
      angleToFace = 120.0;
    } else if (swerveDrivetrain.getAngle() >= -150 && swerveDrivetrain.getAngle() <= -90.0) {
      angleToFace = -120.0;
    } else if (swerveDrivetrain.getAngle() >= 150 || swerveDrivetrain.getAngle() <= -150) {
      angleToFace = 180.0;
    } else if (swerveDrivetrain.getAngle() >= -90.0 && swerveDrivetrain.getAngle() <= -30.0) {
      angleToFace = -60.0;
    }

    Logger.recordOutput("Swerve angle to face", angleToFace);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrivetrain.driveFieldCentricFacingAngle(
        x * UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1)),
        x * UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftX(), 0.1)), angleToFace);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(MathUtil.applyDeadband(driverController.getRightX(), 0.1)) > 0.0;
  }
}
