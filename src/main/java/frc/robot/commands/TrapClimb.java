// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Linkage;

public class TrapClimb extends Command {
  private final AmpArm ampArm; 
  private final Climber climber; 
  private final Linkage linkage; 
  private final XboxController operatorCont = new XboxController(1);
  private double climbHeight = -57.0; 
  private double ampSetpoint = 115.0;
  /** Creates a new TrapClimb. */
  public TrapClimb(AmpArm ampArm, Climber climber, Linkage linkage) {
    this.ampArm = ampArm; 
    this.climber = climber; 
    this.linkage = linkage; 
    addRequirements(ampArm, climber, linkage);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ampArm.setWrist(165.0);
    if(Math.abs(ampArm.getWristPosition() - 165.0) < 2.0 ){
      climber.setLeftHeight(climbHeight, 1);
      climber.setRightHeight(climbHeight, 1);
      ampArm.setArm(ampSetpoint, linkage);
    }
    


    boolean leftClimbCheck = Math.abs(climber.getLeftPosition() - climbHeight) < 1.0;
    boolean rightClimbCheck = Math.abs(climber.getRightPosition() - climbHeight) < 1.0;
    
    System.out.println(Math.abs(climber.getLeftPosition() - climbHeight));
    System.out.println(Math.abs(climber.getRightPosition() - climbHeight));

    if(leftClimbCheck && rightClimbCheck) {
      ampArm.runWrist(getWithDeadband(-operatorCont.getRightY()) * 0.1);
    }
    // if(leftClimbCheck && rightClimbCheck){
    //   System.out.println(leftClimbCheck);
    //   System.out.println(rightClimbCheck);
    //   ampArm.runWrist(getWithDeadband(-operatorCont.getRightY()) * 0.1);
    // }else{
    //   ampArm.setWrist(10.0);
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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
