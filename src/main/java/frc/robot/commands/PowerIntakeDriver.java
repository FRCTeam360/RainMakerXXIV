// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.utils.CommandLogger;

public class PowerIntakeDriver extends Command {

  private Intake intake;
  private Flywheel flywheel;
  private boolean neg;

  /** Creates a new ManualIntake. */
  public PowerIntakeDriver(Intake intake, boolean neg, Flywheel flywheel) {
    this.neg = neg;
    this.intake = intake;
    this.flywheel = flywheel;
    addRequirements(intake, flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(neg) {
      intake.run(-.1);
      flywheel.setBothRPM(-500);
    } else {
      intake.run(.3);
    }
    // if(operatorCont.getRightTriggerAxis() > .75) {
    // intake.run(-.5);
    // } else {
    // intake.run(-.15);
    // }

    CommandLogger.logCommandRunning(this);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    flywheel.stop();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
