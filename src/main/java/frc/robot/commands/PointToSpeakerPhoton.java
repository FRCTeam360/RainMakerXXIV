// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Photon;

public class PointToSpeakerPhoton extends Command {
// THIS COMMAND DOESN'T WORK ATM 2/11/24

Photon photon;
  CommandSwerveDrivetrain drive; 
  
  public PointToSpeakerPhoton(Photon photon, CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.photon = photon;
    this.drive = drive; 
    addRequirements(photon, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

             
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {     
    
    
    double currentDriveAngle; 
    boolean hasTarget = photon.getHasTarget();
        System.out.println("Command Target"  + hasTarget);

    if (hasTarget) {
      double angleCamera = photon.getAngle(); //  TODO IS RADAIAnS? SWITCH TO YAW ; 
      currentDriveAngle = drive.getRotation2d().getRadians() ;

      drive.driveFieldCentricFacingAngle(0, 0, currentDriveAngle,angleCamera );//
                  System.out.println("no target available" + (angleCamera - currentDriveAngle) ) ;

    } else if (!hasTarget)  {
      System.out.println("no target available") ;

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
 