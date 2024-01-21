// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class RunShooter extends Command {
  private Shooter shooter;
  private final XboxController operatorCont = new XboxController(1);

  /** Creates a new RunShooter. */
  public RunShooter(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(Math.abs(operatorCont.getLeftY()) > 0.1) {
    //   shooter.runLeft(operatorCont.getLeftY());
    // } else {
    //   shooter.stopLeft();
    // }

    // if(Math.abs(operatorCont.getRightY()) > 0.1) {
    //   shooter.runRight(operatorCont.getRightY());
    // } else {
    //   shooter.stopRight();
    // }

    shooter.runBoth(-0.8, -1.0);
  }
    
  public void end(boolean interrupted) {
    shooter.stopLeft();
    shooter.stopRight();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
