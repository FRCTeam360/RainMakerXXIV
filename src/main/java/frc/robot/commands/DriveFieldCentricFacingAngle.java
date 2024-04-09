// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.UtilMethods;

public class DriveFieldCentricFacingAngle extends Command {
  private final CommandSwerveDrivetrain drivetrain; 
  private double red;
  private XboxController driverController = new XboxController(0);
  private double blue;
  /** Creates a new FieldCentricFacingAngle. */
  public DriveFieldCentricFacingAngle(CommandSwerveDrivetrain drivetrain, double red, double blue) {
    this.drivetrain = drivetrain; 
    this.red = red;
    this.blue = blue;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = 1;
    double angle = blue;
    if(DriverStation.getAlliance().get() == Alliance.Red){
      angle = red;
      x=-1;
    }
    double left = x*UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftX(), 0.1));
    double forward = x*UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1));
    drivetrain.driveFieldCentricFacingAngle(forward, left, angle);
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
