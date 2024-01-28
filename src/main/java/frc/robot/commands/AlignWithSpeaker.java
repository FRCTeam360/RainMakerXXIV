// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Limelight;

public class AlignWithSpeaker extends Command {
  private Limelight lime = new Limelight();
  private CommandSwerveDrivetrain drive;
  private XboxController driver = new XboxController(0);
  private enum VisionCases {NO_TARGET, HAS_TARGET, ALIGNED_WITH_TARGET};
  private VisionCases state = VisionCases.NO_TARGET;
  /** Creates a new AlignWithSpeaker. */
  public AlignWithSpeaker(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    addRequirements(lime, this.drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(state) {
      case NO_TARGET:
        if(lime.getTV() == 1) {
          state = VisionCases.HAS_TARGET;
        }
        drive.fieldOrientedDrive(driver.getLeftX(), driver.getLeftY(), driver.getRightX());
        break;
      case HAS_TARGET:
        if(lime.getTV() == 0) {
          state = VisionCases.NO_TARGET;
        }
        if(lime.getTX() >= -1 && lime.getTX() <= 1) {
          state = VisionCases.ALIGNED_WITH_TARGET;
        }
        if(lime.getTX() > 0) {
          drive.fieldOrientedDrive(driver.getLeftX(), driver.getLeftY(), -.2);
        } else if(lime.getTX() < 0) {
          drive.fieldOrientedDrive(driver.getLeftX(), driver.getLeftY(), .2);
        }
        break;
      case ALIGNED_WITH_TARGET:
        // if(lime.getTV() == 0) {
        //   state = VisionCases.NO_TARGET;
        // } 
        // if(lime.getTX() <= -1 || lime.getTX() >= 1) {
        //   state = VisionCases.HAS_TARGET;
        // }
        isFinished();
<<<<<<< Updated upstream
        break;
=======
        break; 
>>>>>>> Stashed changes
        
      }
    
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
