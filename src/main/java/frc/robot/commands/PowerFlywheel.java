// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel;

<<<<<<< HEAD:src/main/java/frc/robot/commands/PowerShooter.java
public class PowerShooter extends Command {
  private final Shooter shooter;
=======
public class PowerFlywheel extends Command {
  private final Flywheel flywheel = Flywheel.getInstance();
>>>>>>> Woodbot:src/main/java/frc/robot/commands/PowerFlywheel.java
  private final XboxController operatorCont = new XboxController(1);
  // private final SparkPIDController topPidController = flywheel.topPidController;

<<<<<<< HEAD:src/main/java/frc/robot/commands/PowerShooter.java
  /** Creates a new RunShooter. */
  public PowerShooter(Shooter shooter) {
    this.shooter = shooter;
=======
  /** Creates a new Runflywheel. */
  public PowerFlywheel() {
>>>>>>> Woodbot:src/main/java/frc/robot/commands/PowerFlywheel.java
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(Math.abs(operatorCont.getLeftY()) > 0.1) {
    //   flywheel.runLeft(-operatorCont.getLeftY());
    // } else {
    //   flywheel.stopLeft();
    // }

    // if(Math.abs(operatorCont.getRightY()) > 0.1) {
    //   flywheel.runRight(-operatorCont.getRightY());
    // } else {
    //   flywheel.stopRight();
    // }

    flywheel.runBoth(1.0);

  }
    
  public void end(boolean interrupted) {
    // print when it runs
    System.out.println("ENDING");
    flywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
