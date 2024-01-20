// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;

public class RunIntake extends Command {
  enum IntakeCases {CHECK_ROBOT_EMPTY, EXTEND_INTAKE, WAIT_FOR_SENSOR, RETRACT_STOP}; 
  private Linkage linkage = Linkage.getInstance();
  
  private Intake intake = Intake.getInstance();
  private static XboxController operatorCont = new XboxController(1);
  private boolean hasPiece = false;
  private Timer timer = new Timer();
  private Timer sensorTimer = new Timer();
  
  private IntakeCases state = IntakeCases.CHECK_ROBOT_EMPTY;

 
  
  /** Creates a new Java. */
  public RunIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, linkage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch(state){
      case CHECK_ROBOT_EMPTY:
        if(intake.getSensor()) {
          end(true);
        } else {
          state = IntakeCases.EXTEND_INTAKE;
        }
        break;
      case EXTEND_INTAKE:
        intake.run(.5); // we should extend too but idk how we should implement this
        linkage.setAngle(180);
        if(intake.getAmps() > 20 && timer.get() > .25) {
          sensorTimer.start();
          state = IntakeCases.WAIT_FOR_SENSOR;
          
        }
        break;
      case WAIT_FOR_SENSOR:
        intake.run(.15);
        if(sensorTimer.get() > .25) {
          state = IntakeCases.EXTEND_INTAKE;
          sensorTimer.reset();
        } else if(intake.getSensor()) {
          state = IntakeCases.RETRACT_STOP;
        }
        break;
      case RETRACT_STOP:
        end(true);
        break;

    }
    
    // if(pieceTimer.get() == .5) {
    //   intake.stop();
    //   pieceTimer.stop();
    // }
    // if (operatorCont.getRightBumperPressed()) {
    //   intake.run(1.0);
    // } else if (operatorCont.getLeftBumperPressed()) {
    //   intake.run(-1.0);
    // } else {
    //   intake.stop();
    // }
    // if(operatorCont.getRightTriggerAxis() > .75 && intake.getAmps() > 20 && timer.get() > .25) {
    //   hasPiece = true;
    //   // pieceTimer.start();
    // }
    // if(intake.getSpeed() == 0) {
    //   hasPiece = false;
    // }

    // if(hasPiece) {
    //   intake.run(-.15);
    // }
    // else if(operatorCont.getRightTriggerAxis() > .75) {
    //   intake.run(-.5);
    // }
    //  else {
    //   intake.run(-.15);
    // }

      
    //   System.out.println(operatorCont.getRightTriggerAxis());
    //   SmartDashboard.putNumber("Trigger val: ", operatorCont.getRightTriggerAxis());
  }

  // Called once the command ends or is interrupted.
  // this might change bad code :(
  @Override
  public void end(boolean interrupted) {
    sensorTimer.stop();
    sensorTimer.reset();
    timer.stop();
    timer.reset();
    intake.stop();
    linkage.setAngle(43);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
