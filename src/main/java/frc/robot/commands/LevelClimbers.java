// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LevelClimbers extends Command {
  private Climber climber;
  private PIDController rollPidController = new PIDController(0, 0, 0);

  private double kP = 0;
  private double kI = 0;
  private double kD = 0;

  private double roll = 0;
  private double rollHeight = 0;
  private double initialClimb = 4.5; // inches

  /** Creates a new LevelClimbers. */
  public LevelClimbers(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roll = climber.getRoll();
    
    if(Math.abs(roll) > 1.0) {
      rollHeight = rollPidController.calculate(roll);
    }

    climber.setLeftHeight(initialClimb + rollHeight);
    climber.setRightHeight(initialClimb - rollHeight);

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
