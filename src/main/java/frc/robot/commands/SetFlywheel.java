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

public class SetFlywheel extends Command {
  private final Flywheel flywheel = Flywheel.getInstance();
  private final SparkPIDController topPidController = flywheel.topPidController;

  private double kP = 0.0;
  private double kI = 0.0;
  private double kD = 0.0;

  private double currentRPM = 0.0;
  private Timer time = new Timer();
  private boolean isAtTarget;

  /** Creates a new SetFlywheel. */
  public SetFlywheel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);

    topPidController.setP(kP);
    topPidController.setI(kI);
    topPidController.setD(kD);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("p", kP);
    SmartDashboard.putNumber("i", kI);
    SmartDashboard.putNumber("d", kD);
    SmartDashboard.putNumber("Goal RPM", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentRPM = flywheel.getVelocity();

    double p = SmartDashboard.getNumber("p", 0.01);
    double i = SmartDashboard.getNumber("i", 0);
    double d = SmartDashboard.getNumber("d", 0);

    if ((p != kP)) {
      flywheel.topPidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      flywheel.topPidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      flywheel.topPidController.setD(d);
      kD = d;
    }

    double goalRPM = SmartDashboard.getNumber("Goal RPM", 0.0);

    if (currentRPM != goalRPM) {
      System.out.println("updating setpoint: " + goalRPM);
      time.reset();
      time.start();
      isAtTarget = false;
      flywheel.topPidController.setReference(goalRPM, CANSparkBase.ControlType.kVelocity);
    }

    if (goalRPM < flywheel.getVelocity() + 70.0 || goalRPM > flywheel.getVelocity() - 70.0) { //67.0RPM is 1%
      time.stop();
      isAtTarget = true;
    } else {
      isAtTarget = false;
    }
    // processVariable = encoder.getPosition();

    SmartDashboard.putNumber("Goal RPM", goalRPM);
    SmartDashboard.putNumber("Velocity", flywheel.getVelocity());
    SmartDashboard.putNumber("Error", goalRPM - flywheel.getVelocity());
    SmartDashboard.putNumber("Time Elapsed", time.get());
    SmartDashboard.putBoolean("At target", isAtTarget);
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
