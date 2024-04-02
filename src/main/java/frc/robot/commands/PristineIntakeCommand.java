// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Intake;
import frc.robot.utils.CommandLogger;
import frc.robot.subsystems.Linkage;

public class PristineIntakeCommand extends Command {
  private final Intake intake;
  private final Linkage linkage;
  private AmpArm ampArm;
  private double linkageSetpoint;
  private enum NoteState {
    GENESIS,
    NO_NOTE,
    INTAKE_SENSOR,
    SIDE_SENSOR,
    SHOOTER_SENSOR,
    END
  }
  private NoteState state;
  /** Creates a new IntakeCOmmand. */
  public PristineIntakeCommand(Intake intake, Linkage linkage, AmpArm ampArm, double linkageSetpoint) {
    this.intake = intake;
    this.linkage = linkage;
    this.ampArm = ampArm;
    this.linkageSetpoint = linkageSetpoint;
    addRequirements(intake, linkage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
    state = NoteState.GENESIS;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case GENESIS:
        if(!intake.getSideSensor() || !intake.getShooterSensor()) {
          state = NoteState.END;
        } else if(intake.getSideSensor() && intake.getShooterSensor()) {
          state = NoteState.NO_NOTE;
        }
        break;
      case NO_NOTE:
        linkage.setAngle(0.0, ampArm);
        intake.run(1.0);
        if(!intake.getIntakeSensor()) {
          state = NoteState.INTAKE_SENSOR;
        }
        break;
      case INTAKE_SENSOR:
        linkage.setAngle(90.0, ampArm);
        intake.run(.4);
        if(!intake.getSideSensor()) {
          state = NoteState.SIDE_SENSOR;
        }
        break;
      case SIDE_SENSOR:
        intake.run(.15);
        linkage.setAngle(linkageSetpoint, ampArm);
        if(!intake.getShooterSensor()) {
          state = NoteState.SHOOTER_SENSOR;
        }
        break;
      case SHOOTER_SENSOR:
        intake.stop();
        linkage.setAngle(linkageSetpoint, ampArm);
        if(linkage.isAtSetpoint()) {
          state = NoteState.END;
        }
        break;
      case END:
        break;
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    linkage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(state == NoteState.END) {
      return true;
    }
    return false;
  }
}
