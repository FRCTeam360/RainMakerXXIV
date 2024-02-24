// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberPIDTuner extends Command {
  private final Climber climber;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double kFF = 0;
  private double leftError = 0;
  private double rightError = 0;
  private double goalHeight = 85.0;

  /** Creates a new PIDTuner. */
  public ClimberPIDTuner(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("goal height", 85.0);
    SmartDashboard.putNumber("p", 0);
    SmartDashboard.putNumber("i", 0);
    SmartDashboard.putNumber("d", 0);
    SmartDashboard.putNumber("ff", 0);

    SmartDashboard.putNumber("left error", 0);
    SmartDashboard.putNumber("right error", 0);

    SmartDashboard.putNumber("roll", 0);
    SmartDashboard.putNumber("Left Height", 0);
    SmartDashboard.putNumber("Right Height", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double p = SmartDashboard.getNumber("p", 0.0);
    double i = SmartDashboard.getNumber("i", 0.0);
    double d = SmartDashboard.getNumber("d", 0.0);
    double ff = SmartDashboard.getNumber("ff", 0.0);
    goalHeight = SmartDashboard.getNumber("goal height", 85.0);
    leftError = goalHeight - climber.getLeftPosition();
    rightError = goalHeight - climber.getRightPosition();

    if ((p != kP)) {
      kP = p;
    }

    if ((i != kI)) {
      kI = i;
    }

    if ((d != kD)) {
      kD = d;
    }

    if (ff != kFF) {
      kFF = ff;
    }

    climber.updatePIDF(kP, kI, kD, kFF);
    climber.setLeftHeight(goalHeight);
    climber.setRightHeight(goalHeight);

    SmartDashboard.putNumber("p", kP);
    SmartDashboard.putNumber("i", kI);
    SmartDashboard.putNumber("d", kD);
    SmartDashboard.putNumber("ff", kFF);

    SmartDashboard.putNumber("goal height", goalHeight);

    SmartDashboard.putNumber("left error", 0);
    SmartDashboard.putNumber("right error", 0);

    SmartDashboard.putNumber("Left Height", climber.getLeftPosition());
    SmartDashboard.putNumber("Right Height", climber.getRightPosition());
    // SmartDashboard.putNumber("roll", climber.getRoll());
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
