// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TuneSwerveDrive extends Command {
  private CommandSwerveDrivetrain drivetrain;

  private final XboxController driverCont = new XboxController(0);

  private GenericEntry kPEntry;
  private GenericEntry kIEntry;
  private GenericEntry kDEntry;
  private GenericEntry kSEntry;
  private GenericEntry kVEntry;
  private GenericEntry kAEntry;

  private GenericEntry velocityEntry;

  double kP = 0.0;
  double kI = 0.0;
  double kD = 0.0;
  double kS = 0.0;
  double kV = 0.0;
  double kA = 0.0;
  
  /** Creates a new TuneSwerveDrive. */
  public TuneSwerveDrive(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    setupShuffleboard();
    updateGains(true);
  }

  private void setupShuffleboard(){
        ShuffleboardTab tab = Shuffleboard.getTab("Tune Swerve");

        kPEntry = tab.add("kP", 0.0).getEntry();
        kIEntry = tab.add("kI", 0.0).getEntry();
        kDEntry = tab.add("kD", 0.0).getEntry();
        kSEntry = tab.add("kS", 0.0).getEntry();
        kVEntry = tab.add("kV", 0.0).getEntry();
        kAEntry = tab.add("kA", 0.0).getEntry();

        velocityEntry = tab.add("Velocity", 0.0).getEntry();
  }

  private void updateGains(boolean forceUpdate){
    double kP = kPEntry.getDouble(0.0);
    double kI = kIEntry.getDouble(0.0);
    double kD = kDEntry.getDouble(0.0);
    double kS = kSEntry.getDouble(0.0);
    double kV = kVEntry.getDouble(0.0);
    double kA = kAEntry.getDouble(0.0);
    if (forceUpdate || kP != this.kP || kI != this.kI || kD != this.kD || kS != this.kS || kV != this.kV || kA != this.kA){
      drivetrain.updateDriveGains(kP, kI, kD, kS, kV, kA);
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kS = kS;
      this.kV = kV;
      this.kA = kA;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateGains(false);
    double velocity = velocityEntry.getDouble(0.0);
    System.out.println(velocity);

    if(driverCont.getAButton()){
      drivetrain.fieldCentricDrive(0, velocity/Constants.MAX_SPEED_MPS, 0);
    } else if(driverCont.getXButton()){
      drivetrain.fieldCentricDrive(0, -velocity/Constants.MAX_SPEED_MPS, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
