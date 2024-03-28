// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Intake;
import frc.robot.utils.CommandLogger;
import frc.robot.subsystems.Linkage;
import frc.robot.subsystems.Vision;

public class AutoIntakeCOmmand extends Command {
  private final Intake intake;
  private final Linkage linkage;
  private AmpArm ampArm;
  private Vision vision; 
  private boolean stop = false;
  private double setthatguy;
  private double x = 0;
  private boolean bringup = false;
  private boolean retracts;
  private boolean reversing = false; 
  /** Creates a new IntakeCOmmand. */
  public AutoIntakeCOmmand(Intake intake, Linkage linkage, AmpArm ampArm, Vision vision, double setthatguy, boolean retracts) {
    this.intake = intake;
    this.linkage = linkage;
    this.ampArm = ampArm;
    this.vision = vision; 
    this.setthatguy = setthatguy;
    this.retracts = retracts; 
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    bringup = false;
    CommandLogger.logCommandStart(this);
    linkage.setAngle(0.0, ampArm);
    stop = false;
    x=0;
    reversing = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(bringup) {
      linkage.setAngle(setthatguy, ampArm);
    }
    else if(!intake.getIntakeSensor() && !bringup) {
      intake.run(.4);
      // System.out.println("runnin at 90");
    } else {
      intake.run(.5);
      // System.out.println("runnin at .5");
    }
    if(!intake.getSideSensor()) {
      bringup = true; 
      intake.stop();
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
