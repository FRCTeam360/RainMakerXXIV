// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;

public class IntakeCOmmand extends Command {
  private final Intake intake;
  private final Linkage linkage;
  private boolean stop = false;
  private double setthatguy;
  private boolean shouldStow;
  /** Creates a new IntakeCOmmand. */
  public IntakeCOmmand(Intake intake, Linkage linkage, double setthatguy, boolean shouldStow) {
    this.intake = intake;
    this.linkage = linkage;
    this.setthatguy = setthatguy;
    this.shouldStow = shouldStow;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linkage.setAngle(0.0);
    stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.getAmps() > 20 && intake.getVelocity() <= .05) {
      intake.run(-.9);
      System.out.println("runnin at 90");
    } else {
      intake.run(-.5);
      System.out.println("runnin at .5");
    }
    if(!intake.getSideSensor()) {
      stop = true;
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(shouldStow) {
    linkage.setAngle(setthatguy);
    }
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
