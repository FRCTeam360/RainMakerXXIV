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
  private final SparkPIDController topPIDController = flywheel.topPIDController;

  private double kP = 0.00055;
  private double kI = 0.0;
  private double kD = 0.0;
  private double kFF = 0.000152;

  private double goalRPM = 5500.0;

  private Timer time = new Timer();
  private boolean isAtTarget;

  /** Creates a new SetFlywheel. */
  public SetFlywheel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);

    topPIDController.setP(kP);
    topPIDController.setI(kI);
    topPIDController.setD(kD);
    topPIDController.setFF(kFF);

    SmartDashboard.putNumber("p", kP);
    SmartDashboard.putNumber("i", kI);
    SmartDashboard.putNumber("d", kD);
    SmartDashboard.putNumber("ff", kFF);
    SmartDashboard.putNumber("Goal RPM", 0);
    
    SmartDashboard.putNumber("Velocity", 0.0);
    SmartDashboard.putNumber("Error", 0.0);
    SmartDashboard.putNumber("Time Elapsed", 0.0);
    SmartDashboard.putBoolean("At target", isAtTarget);
    SmartDashboard.putNumber("Top - Bottom Error", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double p = SmartDashboard.getNumber("p", 0.0);
    double i = SmartDashboard.getNumber("i", 0.0);
    double d = SmartDashboard.getNumber("d", 0.0);
    double ff = SmartDashboard.getNumber("ff", 0.0);

    if ((p != kP)) {
      flywheel.topPIDController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      flywheel.topPIDController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      flywheel.topPIDController.setD(d);
      kD = d;
    }

    if (ff != kFF) {
      flywheel.topPIDController.setFF(ff);
      kFF = ff;
    }

    double updatedGoalRPM = SmartDashboard.getNumber("Goal RPM", 0.0);

    if (this.goalRPM != updatedGoalRPM) {
      System.out.println("updating setpoint: " + updatedGoalRPM);
      time.reset();
      time.start();
      isAtTarget = false;
      flywheel.topPIDController.setReference(updatedGoalRPM, CANSparkBase.ControlType.kVelocity);
      goalRPM = updatedGoalRPM;
    }

    if (goalRPM < flywheel.getTopVelocity() + 70.0 || goalRPM > flywheel.getTopVelocity() - 70.0) { // 67.0RPM is 1%
      time.stop();
      isAtTarget = true;
    } else {
      isAtTarget = false;
    }
    // processVariable = encoder.getPosition();

    SmartDashboard.putNumber("Velocity", flywheel.getTopVelocity());
    SmartDashboard.putNumber("Error", goalRPM - flywheel.getTopVelocity());
    SmartDashboard.putNumber("Time Elapsed", time.get());
    SmartDashboard.putBoolean("At target", isAtTarget);
    SmartDashboard.putNumber("Top - Bottom Error", flywheel.getTopVelocity() - flywheel.getBottomVelocity());
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
