// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Intake;
import frc.robot.utils.CommandLogger;
import frc.robot.subsystems.Linkage;

public class JujuIntake extends Command {
  private final Intake intake;
  private final Linkage linkage;
  private AmpArm ampArm;
  private boolean stop = false;
  private double setthatguy;
  private boolean retracts;
  private boolean noNote;
  /** Creates a new IntakeCOmmand. */
  public JujuIntake(Intake intake, Linkage linkage, AmpArm ampArm, double setthatguy, boolean retracts) {
    this.intake = intake;
    this.linkage = linkage;
    this.ampArm = ampArm;
    this.setthatguy = setthatguy;
    this.retracts = retracts;
    noNote = true;
    addRequirements(intake, linkage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
    linkage.setAngle(0.0, ampArm);
    stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.getSideSensor() && intake.getHighSensor() && noNote) {
      intake.run(.9);
    } else if(!intake.getSideSensor() && intake.getHighSensor()) {
      intake.run(.5);
      noNote = false;
    } else if(intake.getSideSensor() && !intake.getHighSensor()) {
      stop = true;
    }
    else if(intake.getSideSensor() && intake.getHighSensor()) {
      intake.run(.2);
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(retracts) {
    linkage.setAngle(setthatguy, ampArm);
    }
    intake.stop();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
