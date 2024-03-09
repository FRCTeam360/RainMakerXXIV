// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Linkage;

public class RydarsSpinup extends Command {
  private final Linkage linkage;
  private final AmpArm arm;
  private final Flywheel flywheel;
  private double angle;
  private double rpm;
  /** Creates a new RydarsSpinup. */
  public RydarsSpinup(Linkage linkage, AmpArm arm, Flywheel flywheel, double angle, double rpm) {
    this.linkage = linkage;
    this.arm = arm;
    this.flywheel = flywheel;
    this.angle = angle;
    this.rpm = rpm;
    addRequirements(linkage, flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheel.setBothRPM(rpm);
    linkage.setAngle(angle, arm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flywheel.stop();
    linkage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
