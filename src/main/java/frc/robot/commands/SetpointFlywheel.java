// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

public class SetpointFlywheel extends Command {
<<<<<<< HEAD
  private double setpoint;
  private Shooter shooter;
=======
  private Flywheel flywheel;
  private double setpointRpm;
>>>>>>> Woodbot

  /** Creates a new SetpointFlywheel. */
  public SetpointFlywheel(Flywheel flywheel, double setpointRpm) {
    this.flywheel = flywheel;
    this.setpointRpm = setpointRpm;
    addRequirements(this.flywheel);

<<<<<<< HEAD
  public SetpointFlywheel(double setpointVelocity, Shooter shooter) {
    this.shooter = shooter;
    setpoint = setpointVelocity;
=======
    // Use addRequirements() here to declare subsystem dependencies.
>>>>>>> Woodbot
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    flywheel.setBothRPM(setpointRpm);

    // if ((p != kP)) {
    // flywheel.leftPidController.setP(p);
    // flywheel.rightPidController.setP(p);
    // kP = p;
    // }
    // if ((i != kI)) {
    // flywheel.leftPidController.setI(i);
    // flywheel.rightPidController.setI(i);
    // kI = i;
    // }
    // if ((d != kD)) {
    // flywheel.leftPidController.setD(d);
    // flywheel.rightPidController.setD(d);
    // kD = d;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    flywheel.stop();
    return false;
  }
}
*/