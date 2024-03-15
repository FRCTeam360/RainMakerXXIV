// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.utils.CommandLogger;

public class 
PowerIntakeReversed extends Command {
  private XboxController operatorCont = new XboxController(1);
  private Intake intake;
  private Lights lights; 
  /** Creates a new RunIntakeReversed. */
  public PowerIntakeReversed(Intake intake, Lights lights) {
    this.intake = intake;
    this.lights = lights; 
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.run(-1.0);
    if(!intake.getSideSensor()){
      // lights.hasNote();
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
