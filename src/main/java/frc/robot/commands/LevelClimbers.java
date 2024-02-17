// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LevelClimbers extends Command {
  private Climber climber;
  private CommandSwerveDrivetrain driveTrain;

  private Pigeon2 pigeon;
  private double roll = 0;
  private double heightOffset;
  private double rollScalar = 0;
  private double climbPose = 4.5; //inches

  /** Creates a new LevelClimbers. */
  public LevelClimbers(Climber climber, CommandSwerveDrivetrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.driveTrain = driveTrain;
    pigeon = driveTrain.getPigeon2();
    heightOffset = climber.heightOffset;;
    addRequirements(this.climber, this.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    roll = pigeon.getRoll().getValueAsDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(roll) > 1.0) {
      heightOffset += Math.signum(roll) * rollScalar;
    }

    climber.runLeft(climbPose - heightOffset);
    climber.runRight(climbPose + heightOffset);
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
