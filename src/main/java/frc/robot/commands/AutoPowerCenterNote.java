package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class AutoPowerCenterNote extends Command{
    enum IntakeCases {CHECK_ROBOT_EMPTY, EXTEND_INTAKE, WAIT_FOR_SENSOR, UP_TO_SHOOTER, DOWN_TO_INTAKE, RETRACT_STOP, BACK_UP, CENTER, DONE}; 
  private final Linkage linkage;
  private double setpoint;
  private final Flywheel flywheel; 
  //private DigitalInput sensor = new DigitalInput(0);
  
  private final Intake intake;
  private final AmpArm arm;
  private static XboxController operatorCont = new XboxController(1);
  private Timer timer = new Timer();
  private double settyspaghettipoint;
  private Timer sensorTimer = new Timer();
  
  private IntakeCases state = IntakeCases.CHECK_ROBOT_EMPTY;
 
  
  /** Creates a new Java. */
  public AutoPowerCenterNote(AmpArm arm, Intake intake, Linkage linkage, Flywheel flywheel, double settyspaghettipoint) {
    this.intake = intake;
    this.settyspaghettipoint = settyspaghettipoint;
    this.linkage = linkage;
    this.flywheel = flywheel; 
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, linkage);
  }

  //   public RunExtendIntake(boolean isAuto) {
  //   inAuto = isAuto;
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   addRequirements(intake);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    CommandLogger.logCommandStart(this);
    linkage.enableBrakeMode();
    timer.reset();
    sensorTimer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(state);
    switch(state){
      case CHECK_ROBOT_EMPTY:
     Logger.recordOutput("INTAKE_STATE", 1);
        if(!intake.getSideSensor()) {
          state = IntakeCases.RETRACT_STOP;
        } else {
          state = IntakeCases.EXTEND_INTAKE;
        }
        break;
      case EXTEND_INTAKE:
           Logger.recordOutput("INTAKE_STATE", 2);

        linkage.setAngle(0.0);
        intake.run(.5);
        if(!intake.getSideSensor()) {
          state = IntakeCases.UP_TO_SHOOTER;
        }


        // we should extend too but idk how we should implement this
        // //linkage.setAngle(180);
        // if(intake.getAmps() > 20 && timer.get() > .25) {
        //   sensorTimer.start();
        //   state = IntakeCases.WAIT_FOR_SENSOR;
        // }
        break;
      case UP_TO_SHOOTER:
        intake.run(.3);
        if(intake.getDiagonalSensor()) {
          state = IntakeCases.BACK_UP;
        }
        // if(!intake.get[\][]\HighSensor()){
        //     state = IntakeCases.RETRACT_STOP;
        // }
        break;
      case BACK_UP:
        linkage.setAngle(90.0);
        intake.run(-.3);
        if(intake.getSideSensor()){
            state = IntakeCases.CENTER;
            sensorTimer.start();
        }
        break;
      case CENTER:
        linkage.setAngle(90.0);
        if(sensorTimer.get() > 2) {
          intake.run(.3);
        } else {
          intake.run(.2);
        }
        if(!intake.getDiagonalSensor()){
          intake.stop();
          state = IntakeCases.DONE;
        }
        // }else{
        //   flywheel.setBothRPM(setpoint);;
        // }
        break;
      case DONE:
        linkage.setAngle(settyspaghettipoint); 
        if(linkage.isAtSetpoint()) {
          state = IntakeCases.RETRACT_STOP;
        }   
    case RETRACT_STOP:
Logger.recordOutput("INTAKE_STATE", 6);

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
