// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class IntakeForevsRun extends Command {
  private Intake intake;
  private boolean stop;
  private Linkage linkage;
  private Flywheel flywheel;
  private AmpArm amparm;
  private boolean IGOTIT;
  /** Creates a new IntakeForevsRun. */
  public IntakeForevsRun(Intake intake, Flywheel flywheel, Linkage linkage, AmpArm amparm) {
    this.intake = intake;
    this.flywheel = flywheel;
    this.linkage = linkage;
    this.amparm = amparm;
    addRequirements(intake, linkage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    linkage.setAngle(0.0, amparm);
    CommandLogger.logCommandStart(this);
    IGOTIT = false;
    stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!intake.getSideSensor() || !intake.getShooterSensor()) {
      IGOTIT = true;
    } 
    if(flywheel.isAtSetpoint()) {
    intake.run(1.0);
    }
    if(intake.getSideSensor() && intake.getShooterSensor() && IGOTIT) {
      stop = true;
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    linkage.setAngle(0.0, amparm);
    CommandLogger.logCommandEnd(this);
    intake.stop();
    System.out.println("IT IS FINISHED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stop;
  }
}
