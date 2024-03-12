// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Linkage;

public class ShootingPrepRyRy extends Command {
  private final Linkage linkage;
  private final Flywheel flywheel;
  private final AmpArm arm;
  private double link;
  private double fly;
  /** Creates a new ShootingPrepRyRy. */
  public ShootingPrepRyRy(Linkage linkage, Flywheel flywheel, AmpArm arm, double link, double fly) {
    this.linkage = linkage;
    this.flywheel = flywheel;
    this.arm = arm;
    this.link = link;
    this.fly = fly;
    addRequirements(linkage, flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    linkage.setAngle(link, arm);
    flywheel.setBothRPM(fly);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    linkage.stop();
    flywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
