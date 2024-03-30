// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class PowerAmpArm extends Command {
  private final AmpArm ampArm;
  private final Linkage linkage;
  private final XboxController operatorCont = new XboxController(1);
  private final XboxController testCont = new XboxController(2);
  private double position;

  /** Creates a new PowerArm. */
  public PowerAmpArm(AmpArm ampArm, Linkage linkage) {
    this.ampArm = ampArm;
    this.linkage = linkage;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ampArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = ampArm.getArmPosition();
    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double operatorContRightY = getWithDeadband(-operatorCont.getRightY() * 0.1);
    double testContRightY = getWithDeadband(-testCont.getRightY() * 0.1);
    if(operatorContRightY != 0){
      ampArm.runWrist(operatorContRightY);
    } else if(testContRightY != 0){
      ampArm.runWrist(testContRightY);
    }else{
      ampArm.runWrist(0.0);
    }
    
    
    //ampArm.runArm(getWithDeadband(operatorCont.getLeftY()) * -0.5, linkage);
    System.out.println("position = " + position);

    if (Math.abs(operatorCont.getLeftY()) > 0.1) {
      System.out.println("manual");
      ampArm.runArm(getWithDeadband(operatorCont.getLeftY()) * -0.5, linkage);
      position = ampArm.getArmPosition();
    }else if(Math.abs(testCont.getLeftY()) > 0.1){
      System.out.println("manual");
      ampArm.runArm(getWithDeadband(testCont.getLeftY()) * -0.5, linkage);
      position = ampArm.getArmPosition();
    } else {
      System.out.println("setpoint");
      ampArm.setArm(position, linkage);
    }

    // FOR CHARLIE
    // if (operatorCont.getBackButton()) {
    //   ampArm.runArm(0.3, linkage);
    // } else if (operatorCont.getLeftStickButton()) {
    //   ampArm.runArm(-0.3, linkage);
    // }

    // if (operatorCont.getStartButton()) {
    //   ampArm.runWrist(0.3);
    // } else if (operatorCont.getRightStickButton()) {
    //   ampArm.runWrist(-0.3);
    // }

  }

  public double getWithDeadband(double input) {
    if (Math.abs(input) < 0.1) {
      input = 0.0;
    }
    return input;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampArm.stopArm();
    ampArm.stopWrist();
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
