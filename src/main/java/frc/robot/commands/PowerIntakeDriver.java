// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utils.CommandLogger;

public class PowerIntakeDriver extends Command {
  private XboxController operatorCont = new XboxController(1);

  private Intake intake;

  /** Creates a new ManualIntake. */
  public PowerIntakeDriver(Intake intake) {
    addRequirements(intake);
    this.intake = intake;
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
    // if(operatorCont.getRightTriggerAxis() > .75) {
    // intake.run(-.5);
    // } else {
    // intake.run(-.15);
    // }

    if (operatorCont.getRightTriggerAxis() > 0.1) {
      intake.run(operatorCont.getRightTriggerAxis());
    } else {
      intake.stop();
    }

    CommandLogger.logCommandRunning(this);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
