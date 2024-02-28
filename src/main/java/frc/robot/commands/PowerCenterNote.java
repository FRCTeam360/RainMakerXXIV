package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandLogger;

public class PowerCenterNote extends Command{
    enum IntakeCases {CHECK_ROBOT_EMPTY, EXTEND_INTAKE, WAIT_FOR_SENSOR, UP_TO_SHOOTER, DOWN_TO_INTAKE, RETRACT_STOP, BACK_UP, CENTER, DONE}; 
  private final Linkage linkage;
  private double setpoint;
  //private DigitalInput sensor = new DigitalInput(0);
  
  private final Intake intake;
  private final AmpArm arm;
  private static XboxController operatorCont = new XboxController(1);
  private Timer timer = new Timer();
  private Timer sensorTimer = new Timer();
  
  private IntakeCases state = IntakeCases.CHECK_ROBOT_EMPTY;

 
  
  /** Creates a new Java. */
  public PowerCenterNote(Intake intake, Linkage linkage, AmpArm arm) {
    this.intake = intake;
    this.linkage = linkage;
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
        linkage.setAngle(0.0, arm);                           
        intake.run(1);
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
        intake.run(.4);
        linkage.setAngle(130.0, arm);
        if(!intake.getHighSensor()){
            state = IntakeCases.BACK_UP;
        }
        break;
      case BACK_UP:
        intake.run(-.2);
        if(intake.getSideSensor()){
            state = IntakeCases.CENTER;
        }
        break;
      case CENTER:
        intake.run(.2);
        if(!intake.getHighSensor()){
            state = IntakeCases.RETRACT_STOP;
        }
        break;
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
