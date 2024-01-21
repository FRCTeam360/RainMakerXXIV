// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TunerXDrive extends Command {
  private final XboxController driverCont = new XboxController(0);

  private final CommandSwerveDrivetrain driveTrain = TunerConstants.DriveTrain;
  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MAX_SPEED * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  // driving in open loop

  /** Creates a new TunerXDrive. */
  public TunerXDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private double getYWithDeadzone() {
    if (Math.abs(driverCont.getLeftX()) >= 0.125 || Math.abs(driverCont.getLeftY()) >= 0.125) {
        return driverCont.getLeftY();
    } else {
        return 0.0;
    }
}

private double getXWithDeadzone() {
    if (Math.abs(driverCont.getLeftX()) >= 0.125 || Math.abs(driverCont.getLeftY()) >= 0.125) {
        return driverCont.getLeftX();
    } else {
        return 0.0;
    }
}

public double getAlignmentAngularVelocity() {
    double currentRadians = driveTrain.getGyroscopeRotation().getRadians();
    double desiredRadians = (Math.PI / 2) * (double) Math.round(currentRadians / (Math.PI / 2));
    double error = desiredRadians-currentRadians;
    if (Math.abs(error) < 0.01) {
        return 0.0;
    } else if (currentRadians > desiredRadians) {
        return -0.5;
    } else {
        return 0.5;
    }
}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drivetrain.applyRequest(
        () -> drive.withVelocityX(MathUtil.applyDeadband(-driverCont.getLeftY(), 0.1) * Constants.MAX_SPEED) // drive
                                                                                                             // forward
                                                                                                             // with
                                                                                                             // negative
                                                                                                             // y
            // negative Y (forward)
            .withVelocityY(MathUtil.applyDeadband(-driverCont.getLeftX(), 0.1) * Constants.MAX_SPEED) // drive left with
                                                                                                      // negative x
            .withRotationalRate(MathUtil.applyDeadband(-driverCont.getRightX(), 0.1) * Constants.MAX_ANGULAR_RATE) // drive
                                                                                                                   // counterclockwise
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // x
    );
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
