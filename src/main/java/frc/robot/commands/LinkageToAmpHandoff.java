// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.AmpIntake;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;

public class LinkageToAmpHandoff extends Command {
  private final Linkage linkage;
  private final AmpArm ampArm;
  private final AmpIntake ampIntake;
  private final Flywheel flywheel;
  private final Intake intake;

  private boolean done;

  private States state;

  //private double ampThreshold;

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
    state = States.LINKAGE_DOWN;
    timer.reset();
    done = false;
    //ampThreshold = 20;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //drivetrain.getRobotRelativeSpeeds() > 0.1

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
        intake.run(0.7);
        flywheel.handoff(250.0);
        ampIntake.runIntake(0.70);
        if (timer.get() > 0.2) {
          if (ampIntake.getAmps() > 20) {
            ampIntake.stop();
            state = States.HAS_NOTE;
          }
        }
        break;
      case HAS_NOTE:
        ampArm.setArm(5.0, linkage);
        if (Math.abs(ampArm.getArmPosition()) < 1.0) {
          state = States.RETRACTED;
        }
        break;
      case RETRACTED:
        linkage.setAngle(174.0, ampArm);
        System.out.println(linkage.getAngle());
        if (Math.abs(linkage.getAngle() - 174) < 1.0) {
          done = true;
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampIntake.stop();
    flywheel.stop();
    intake.stop();
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}