// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class DiagonalSensorIntake extends Command {
  private final Flywheel flywheel;
  private final Intake intake;
  private final Linkage linkage;
  private final AmpArm arm;

  private double x = 0;
  private double flywheelSetpoint;
  private enum IntakeCases {
    EXTEND_INTAKE,
    MOVE_UP_INTAKE,
    REVERSE_INTAKE,
    SPIN_UP_FLYWHEEL,
    END
  }
  private IntakeCases state;
  /** Creates a new DiagonalSensorIntake. */
  public DiagonalSensorIntake(AmpArm arm, Flywheel flywheel, Intake intake, Linkage linkage, double flywheelSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    state = IntakeCases.EXTEND_INTAKE;
    this.flywheel = flywheel;
    this.intake = intake;
    this.linkage = linkage;
    this.arm = arm;
    this.flywheelSetpoint = flywheelSetpoint;
    addRequirements(flywheel, intake, linkage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { //DONT USE I THIN? NO DIAGONAL SENSOR ANYMORE
    CommandLogger.logCommandStart(this);
    state = IntakeCases.EXTEND_INTAKE;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(intake.getAmps() > 20 && intake.getVelocity() <= .05) {
    //   this.x = .1;
    // } else {
    //   this.x = 0;
    // }
    switch(state) {
      case EXTEND_INTAKE:
        linkage.setAngle(0.0, arm);
        intake.run(.4);
        if(!intake.getDiagonalSensor()) {
          state = IntakeCases.SPIN_UP_FLYWHEEL;
        }
        break;
      case MOVE_UP_INTAKE:
        linkage.setAngle(90.0, arm);
        intake.run(.5);
        if(!intake.getDiagonalSensor()) {
          state = IntakeCases.REVERSE_INTAKE;
        }
        break;
      case REVERSE_INTAKE:
        linkage.setAngle(90.0, arm);
        intake.run(-.5);
        if(intake.getSideSensor()) {
          state = IntakeCases.SPIN_UP_FLYWHEEL;
        }
        break;
      case SPIN_UP_FLYWHEEL:
        linkage.setAngle(90.0, arm);
        if(linkage.isAtSetpoint()) {
          state = IntakeCases.END;
        }
        break;
      case END:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    linkage.stop();
    intake.stop();
    flywheel.stop();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(state == IntakeCases.END) {
      return true;
    }
    return false;
  }
}
