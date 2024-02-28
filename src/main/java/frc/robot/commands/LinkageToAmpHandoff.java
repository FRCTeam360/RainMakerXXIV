// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.AmpIntake;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class LinkageToAmpHandoff extends Command {
  private final Linkage linkage;
  private final AmpArm ampArm;
  private final AmpIntake ampIntake;
  private final Flywheel flywheel;
  private final Intake intake;

  private boolean done;

  private States state;
  
  private final Timer timer = new Timer();

  private enum States {
    LINKAGE_DOWN, AMP_ARM_UP, INTAKING, HAS_NOTE, RETRACTED, SET_ARM
  }

  
  /** Creates a new LinkageToAmpHandoff. */
  public LinkageToAmpHandoff(Linkage linkage, AmpArm ampArm, AmpIntake ampIntake, Flywheel flywheel, Intake intake) {
    this.linkage = linkage;
    this.ampArm = ampArm;
    this.ampIntake = ampIntake;
    this.flywheel = flywheel;
    this.intake = intake;
    
    addRequirements(linkage, ampArm, ampIntake, flywheel, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
    state = States.LINKAGE_DOWN;
    timer.reset();
    done = false;
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(state);
    switch (state) {
      case LINKAGE_DOWN:
        linkage.setAngle(0.0, ampArm);
        if (linkage.getAngle() < 2.0) {
          state = States.SET_ARM;
        }
        break;
        case SET_ARM:
        ampArm.setArm(-45.0, linkage);
        ampArm.setWrist(45.0);
        if (Math.abs(ampArm.getArmPosition() + 45.0) < 2.0 && Math.abs(ampArm.getWristPosition() - 45.0) < 2.0) {
          timer.start();
          state = States.INTAKING;
        }
        break;
      case INTAKING:
        intake.run(0.5);
        flywheel.handoff(750.0);
        ampIntake.runIntake(0.5);
        if (timer.get() > 0.1) {
          if (ampIntake.getAmps() > 20) {
            ampIntake.stop();
            state = States.HAS_NOTE;
          }
        }
        break;
      case HAS_NOTE:
        ampArm.setArm(0.0, linkage);
        if (Math.abs(ampArm.getArmPosition()) < 1.0) {
          state = States.RETRACTED;
        }
        break;
      case RETRACTED:
        linkage.setAngle(174.0, ampArm);
        done = true;
        break;
    }

    // System.out.println(state);

    // boolean link = false;
    // boolean wrist = false;
    // boolean arm = false;
    // boolean wheel = false;

    // switch (state) {
    // case NO_NOTE:
    // ampArm.setArm(-39.4);
    // ampArm.setWrist(57.4);
    // ampIntake.runIntake(0.2);
    // linkage.setAngle(25.0);
    // flywheel.handoff(1800.0);

    // link = Math.abs(linkage.getAngle() - 25.0) < 3.0;
    // wrist = Math.abs(ampArm.getWristPosition() - 57.4) < 2.0;
    // arm = Math.abs(ampArm.getArmPosition() + 39.4) < 2.0;
    // wheel = Math.abs(flywheel.getLeftVelocity() - 1800.0) < 20.0;

    // System.out.println("link = " + link + " wrist = " + wrist + " arm = " + arm);
    // System.out.println("arm angle" + ampArm.getArmPosition());
    // if (link && wrist && arm && wheel) {
    // state = States.RUN_INTAKE;
    // }
    // break;
    // case RUN_INTAKE:
    // ampArm.setArm(-39.4);
    // ampArm.setWrist(57.4);
    // ampIntake.runIntake(1.0);
    // linkage.setAngle(25.0);
    // flywheel.handoff(1800.0);
    // intake.run(0.2);
    // if (ampIntake.getAmps() > 25) {
    // ampIntake.stop();
    // flywheel.stop();
    // state = States.NOTE_CENTERED;
    // }
    // break;
    // case NOTE_CENTERED:
    // flywheel.stop();

    // done = true;
    // break;
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampIntake.stop();
    flywheel.stop();
    intake.stop();
    timer.stop();
    timer.reset();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
