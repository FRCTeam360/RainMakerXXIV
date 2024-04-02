// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;
import frc.robot.utils.UtilMethods;

public class FieldOrientedDrive extends Command {
  private final XboxController driverController = new XboxController(0);

  private final CommandSwerveDrivetrain driveTrain;
  private final Linkage linkage;
  private final AmpArm ampArm;
  private double x = 1;
  private boolean isSlow;

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MAX_SPEED_MPS).withRotationalDeadband(Constants.MAX_ANGULAR_RATE)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  /** Creates a new TunerXDrive. */
  public FieldOrientedDrive(CommandSwerveDrivetrain driveTrain, Linkage linkage, AmpArm ampArm, boolean isSlow) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.linkage = linkage;
    this.ampArm = ampArm;
    this.isSlow = isSlow;
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {

    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      x = -1;
    } else {
      x = 1;
    }
    boolean isSlowDrive = linkage.getAngle() < 10.0 || ampArm.getArmPosition() > 45.0 || isSlow;
    Logger.recordOutput("Swerve: is slow", isSlowDrive);
    if (linkage.getAngle() < 10.0 || ampArm.getArmPosition() > 45.0 || isSlow) {
      driveTrain.fieldCentricDrive(
          x * UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftX() * .5, 0.1)),
          x * UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftY() * .5, 0.1)),
          UtilMethods.squareInput(MathUtil.applyDeadband(driverController.getRightX() * .75, 0.1)),
          Constants.MAX_SPEED_MPS, Constants.MAX_ANGULAR_RATE);
    } else {
      driveTrain.fieldCentricDrive(
          x * UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftX(), 0.1)),
          x * UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1)),
          UtilMethods.squareInput(MathUtil.applyDeadband(driverController.getRightX(), 0.1)),
          Constants.MAX_SPEED_MPS, Constants.MAX_ANGULAR_RATE);
    }

    CommandLogger.logCommandRunning(this);
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
