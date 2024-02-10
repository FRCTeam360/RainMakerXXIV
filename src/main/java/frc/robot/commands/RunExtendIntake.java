// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hardware.LinkageIOTalonFX;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class RunExtendIntake extends Command {
  enum IntakeCases {CHECK_ROBOT_EMPTY, EXTEND_INTAKE, WAIT_FOR_SENSOR, UP_TO_SHOOTER_P1, UP_TO_SHOOTER_P2, RETRACT_STOP}; 
  private Linkage linkage;
  //private DigitalInput sensor = new DigitalInput(0);
  
  private Intake intake;
  private static XboxController operatorCont = new XboxController(1);
  private Timer timer = new Timer();
  private Timer sensorTimer = new Timer();
  private double setPoint;
  
  private IntakeCases state = IntakeCases.CHECK_ROBOT_EMPTY;

 
  
  /** Creates a new Java. */
  public RunExtendIntake(Intake intake) {
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  //   public RunExtendIntake(boolean isAuto) {
  //   inAuto = isAuto;
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   addRequirements(intake);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    CommandLogger.logCommandStart(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(state);
    switch(state){
      case CHECK_ROBOT_EMPTY:
        if(intake.getSideSensor()) {
          state = IntakeCases.EXTEND_INTAKE;
        }
        break;
      case EXTEND_INTAKE:
        intake.run(.45); // we should extend too but idk how we should implement this
        //linkage.setAngle(180);
        if(intake.getAmps() > 20 && timer.get() > .25) {
          sensorTimer.start();
          state = IntakeCases.WAIT_FOR_SENSOR;
        }
        break;
      case WAIT_FOR_SENSOR:
        intake.run(.225);
        if(!intake.getHighSensor()) {
          state = IntakeCases.UP_TO_SHOOTER_P1;
        }
        // if(sensorTimer.get() > 1) {
        //   state = IntakeCases.EXTEND_INTAKE;
        //   sensorTimer.reset();
        // } 
        // if(!intake.getSideSensor()){
        //  // setPoint = intake.encoder.getPosition() + 1.28436279297;
        //   state = IntakeCases.RETRACT_STOP;
        // }
        break;
      case UP_TO_SHOOTER_P1:
        // if(!intake.getHighSensor()) {
        //   state = IntakeCases.UP_TO_SHOOTER_P2;
        // } else {
        //   intake.run(.25);
        // }
        // if(intake.getSideSensor()) {
        //   state = IntakeCases.RETRACT_STOP;
        // }
        if(intake.getHighSensor()) {
          state = IntakeCases.UP_TO_SHOOTER_P2;
        }
      
        break;
      case UP_TO_SHOOTER_P2:
          if(!intake.getHighSensor()) {
            state = IntakeCases.RETRACT_STOP;
          } else {
            intake.run(-.14);
          }
      case RETRACT_STOP:
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
    CommandLogger.logCommandRunning(this);
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
    state = IntakeCases.CHECK_ROBOT_EMPTY;
    //linkage.setAngle(43);
    CommandLogger.logCommandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(state == IntakeCases.RETRACT_STOP) {
      System.out.print("COMMAND ENDED");
      return true;
    }
    return false;
  }
}
