// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Vision;
import frc.robot.utils.UtilMethods;

public class PointDrivebaseAtTarget extends Command {
  private final XboxController driverController = new XboxController(0);

  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final Lights lights;


  /** Creates a new pointDrivebaseatTarget. */
  public PointDrivebaseAtTarget(CommandSwerveDrivetrain drivetrain, Vision vision, Lights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.lights = lights;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setPastTX(100.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x =1;
    if(DriverStation.getAlliance().get() == Alliance.Red) {
      x= -1;
    }
    double left = x*UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftX(), 0.1));
    double forward = x*UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1));
    double rotation = UtilMethods.squareInput(MathUtil.applyDeadband(driverController.getRightX(), 0.1));

    // Points the drivebase at the target
    if (Objects.nonNull(vision) && vision.isTargetInView()) {
      drivetrain.pointAtTarget(forward, left, vision.getTX());
      // lights.limelight();
    } else{
      drivetrain.fieldCentricDrive(left, forward, rotation);
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
