// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.UtilMethods;

public class TrapSetUp extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Linkage linkage; 
  private final AmpArm ampArm; 
  private final Climber climber; 
  private Timer time = new Timer();
  /** Creates a new Trappin. */
  public TrapSetUp(CommandSwerveDrivetrain drivetrain, Linkage linkage, AmpArm ampArm, Climber climber) {
    this.linkage = linkage;
    this.ampArm = ampArm;  
    this.climber = climber; 
    this.drivetrain = drivetrain;
    addRequirements(drivetrain, linkage, ampArm, climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    linkage.setAngle(0.0, ampArm);
    if(time.get() < .3) {
      drivetrain.robotCentricDrive(
      0,
      .06,
     0);
     ampArm.setArm(90, linkage);
     ampArm.setWrist(15.0);
     climber.setLeftHeight(40);
     climber.setRightHeight(40);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
    time.reset();
    drivetrain.robotCentricDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(time.get() > .4) {
      return true;
    }
    return false;
  }
}
