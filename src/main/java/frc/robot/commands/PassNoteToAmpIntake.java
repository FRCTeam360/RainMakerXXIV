// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.AmpIntake;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;

public class PassNoteToAmpIntake extends Command {
  private final Flywheel flywheel;
  private final Intake intake;
  private final Linkage linkage;
  private final AmpArm ampArm;
  private final AmpIntake ampIntake;

  private enum PassyStates {
    INTAKE_GOT_THAT_THANG,
    THROW_IT_BACK,
    OMG_GIRLIE_I_GOT_IT,
    WERE_SO_OVER
  }
  PassyStates state = PassyStates.INTAKE_GOT_THAT_THANG;
  /** Creates a new PassNoteToAmpIntake. */
  public PassNoteToAmpIntake(Flywheel flywheel, Intake intake, Linkage linkage, AmpArm ampArm, AmpIntake ampIntake, double linkageAngle, double flywheelRPM) {
    this.flywheel = flywheel;
    this.intake = intake;
    this.linkage = linkage;
    this.ampArm = ampArm;
    this.ampIntake = ampIntake;
    addRequirements(flywheel, intake, linkage, ampArm, ampIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case INTAKE_GOT_THAT_THANG:
        linkage.setAngle(linkageAngle);
        ampArm.setArm(armAngle);
        if(linkage.isAtSetpoint()) {
          state = PassyStates.THROW_IT_BACK;
        }
        break;
      case THROW_IT_BACK:
        flywheel.setBothRPM(flywheelRPM);
        if(ampIntake.getAmps() > 25) {
          flywheel.stop();
          state = PassyStates.OMG_GIRLIE_I_GOT_IT;
        }  
        break;  
      case OMG_GIRLIE_I_GOT_IT:
        double intakePosition = ampIntake.getPosition();
        ampIntake.runIntake(0.1);
        if(ampIntake.getPosition() - intakePosition > 2.7) {
          ampIntake.stop();
          state = PassyStates.WERE_SO_OVER;
        }
        break;
      case WERE_SO_OVER:
        break;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(state == PassyStates.WERE_SO_OVER) {
      System.out.print("COMMAND ENDED");
      return true;
    }
    return false;
  }
}
