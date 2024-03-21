// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Linkage;
import frc.robot.subsystems.Vision;
import frc.robot.utils.CommandLogger;

public class SetLinkage extends Command {
  private final Linkage linkage;
  private final AmpArm arm;
  private double setpoint;
  private final Vision vision;
  /** Creates a new SetLinkageTa\
   * lon. */
  public SetLinkage(Linkage linkage, double setpoint, AmpArm arm) {
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    this.linkage = linkage;
    this.arm = arm;
    this.vision = null;
    addRequirements(linkage);
  }

  public SetLinkage(Linkage linkage, double setpoint, AmpArm arm, Vision vision) {
    this.setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
    this.linkage = linkage;
    this.arm = arm;
    this.vision = vision;
    addRequirements(linkage); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
    //SmartDashboard.putNumber("error", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("error", -7.0 - linkage.getAngle());
    if (vision != null && vision.isTargetInView()) {
      setpoint = vision.getLinkageSetpoint();
    }
    linkage.setAngle(setpoint, arm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    linkage.run(0.0, arm);
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
