// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TunerXDrive extends Command {
  private final XboxController driverController = new XboxController(0);
  
  private final CommandSwerveDrivetrain driveTrain = TunerConstants.DriveTrain;
  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.MAX_SPEED_MPS * 0.1).withRotationalDeadband(Constants.MAX_ANGULAR_RATE * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric

  // driving in open loop
  
  /** Creates a new TunerXDrive. */
  public TunerXDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("angle", driveTrain.getAngle());
  }

public double getWithDeadzone(double value) { // not sure if this is what you wanted me to do but i tried :(
  if (Math.abs(value) <= 0.125) {
      return 0.0;
  } else {
      return value;
  }
}


public double getAlignmentAngularVelocity() {
    double currentRadians = Math.toRadians(driveTrain.getAngle());
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

    //TUNERX DRIVE CODE
    driveTrain.setDefaultCommand( // Drivetrain will execute this command periodically
        driveTrain.applyRequest(
            () -> drive.withVelocityX(MathUtil.applyDeadband(driverController.getLeftY(), 0.1) * Constants.MAX_SPEED_MPS) //drive forward with negative y
                // negative Y (forward)
                .withVelocityY(MathUtil.applyDeadband(driverController.getLeftX(), 0.1) * Constants.MAX_SPEED_MPS) // drive left with negative x
                .withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(), 0.1) * Constants.MAX_ANGULAR_RATE) // drive counterclockwise with negative x                                                                                                  
        )
      );


        // if(driverController.getPOV() == 0){
        //     driveTrain.zero();
        // }
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
