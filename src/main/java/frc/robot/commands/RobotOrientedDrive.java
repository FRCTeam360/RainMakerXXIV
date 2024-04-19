
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.CommandLogger;
import frc.robot.utils.UtilMethods;

public class RobotOrientedDrive extends Command {
  private final XboxController driverController = new XboxController(0);
  private double x = 1;

  private final CommandSwerveDrivetrain driveTrain;
  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MAX_SPEED_MPS * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * 0.1) // fix for
                                                                                                            // trap is
                                                                                                            // make the
                                                                                                            // multiplier
                                                                                                            // smaller?
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  /** Creates a new RobotOrientedDrive. */
  public RobotOrientedDrive(CommandSwerveDrivetrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isPresent() && alliance.get() == Alliance.Red) {
      x = -1;
    } else {
      x = 1;
    }

    driveTrain.robotCentricDrive(
        x * UtilMethods.squareInput(MathUtil.applyDeadband(driverController.getLeftX() * .25, 0.1)),
        x * UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftY() * .25, 0.1)),
        x * UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getRightX() * .50, 0.1)));
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
