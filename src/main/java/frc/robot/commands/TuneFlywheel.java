// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class TuneFlywheel extends Command {
  private Flywheel flywheel;

  private double goalRPM = 0.0;


  /** Creates a new SetFlywheel. */
  public TuneFlywheel(Flywheel flywheel) {
    this.flywheel = flywheel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Goal RPM", 0.0);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double updatedGoalRPM = SmartDashboard.getNumber("Goal RPM", 0.0);
    flywheel.setBothRPM(updatedGoalRPM);
    SmartDashboard.putNumber("Left RPM", flywheel.getLeftVelocity());
    SmartDashboard.putNumber("Right RPm", flywheel.getRightVelocity());
    // processVariable = encoder.getPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
