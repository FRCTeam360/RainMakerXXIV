// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* package frc.robot.commands;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetpointFlywheel extends Command {
  private double setpoint;
  private Shooter shooter;

  private double kP = 0.01;
  private double kD = 0.0;
  private double kI = 0.0;
  private double kFF = 0.0;

  public SetpointFlywheel(double setpointVelocity, Shooter shooter) {
    this.shooter = shooter;
    setpoint = setpointVelocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("p", kP);
    SmartDashboard.putNumber("i", kI);
    SmartDashboard.putNumber("d", kD);
    SmartDashboard.putNumber("ff", kFF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double p = SmartDashboard.getNumber("p", 0.01);
    double i = SmartDashboard.getNumber("i", 0);
    double d = SmartDashboard.getNumber("d", 0);
    double ff = SmartDashboard.getNumber("ff", 0);

    if ((p != kP)) {
      shooter.leftPidController.setP(p);
      shooter.rightPidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      shooter.leftPidController.setI(i);
      shooter.rightPidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      shooter.leftPidController.setD(d);
      shooter.rightPidController.setD(d);
      kD = d;
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
*/