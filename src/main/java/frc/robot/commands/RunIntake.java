// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RunIntake extends Command {
  private Intake intake = Intake.getInstance();
  private static XboxController operatorCont = new XboxController(1);
  private boolean hasPiece = false;
  private Timer timer = new Timer();
  private Timer pieceTimer = new Timer();
  
  /** Creates a new Java. */
  public RunIntake() {

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(pieceTimer.get() == .5) {
      intake.stop();
      pieceTimer.stop();
    }
    // if (operatorCont.getRightBumperPressed()) {
    //   intake.run(1.0);
    // } else if (operatorCont.getLeftBumperPressed()) {
    //   intake.run(-1.0);
    // } else {
    //   intake.stop();
    // }
    if(operatorCont.getRightTriggerAxis() > .75 && intake.getAmps() > 20 && timer.get() > .25) {
      hasPiece = true;
      pieceTimer.start();
    }
    if(intake.getSpeed() == 0) {
      hasPiece = false;
    }

    if(hasPiece) {
      intake.run(-.15);
    }
    else if(operatorCont.getRightTriggerAxis() > .75) {
      intake.run(-.5);
    }
     else {
      intake.run(-.15);
    }

      
      System.out.println(operatorCont.getRightTriggerAxis());
      SmartDashboard.putNumber("Trigger val: ", operatorCont.getRightTriggerAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pieceTimer.reset();
    timer.reset();
    timer.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
