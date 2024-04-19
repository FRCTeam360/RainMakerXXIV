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
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.UtilMethods;

public class DefenseFieldOrientedDrive extends Command {
  private final XboxController driverController = new XboxController(0);

  private final CommandSwerveDrivetrain driveTrain;
  private final Linkage linkage;
  private final AmpArm ampArm;
  private double x = 1;

  private double MAX_VELOCITY = 6.0; //meters / sec fr
  private double MAX_ANGULAR = Math.PI * 7;

  /** Creates a new DefenseFieldOrientedDrive. */

  public DefenseFieldOrientedDrive(CommandSwerveDrivetrain driveTrain, Linkage linkage, AmpArm ampArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.linkage = linkage;
    this.ampArm = ampArm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("running defense mode");

    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      x = -1;
    } else {
      x = 1;
    }

    driveTrain.fieldCentricDrive(
        x * UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftX(), 0.1)),
        x * UtilMethods.squareInput(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1)),
        UtilMethods.squareInput(MathUtil.applyDeadband(driverController.getRightX(), 0.1)), 
        this.MAX_VELOCITY, this.MAX_ANGULAR);
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
