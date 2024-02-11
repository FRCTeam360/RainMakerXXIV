// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Photon;

public class pointToSpeakerPhoton extends Command {
  /** Creates a new pointToSpeakerPhoton. */
  Photon photon;
  CommandSwerveDrivetrain drive; 
  
  public pointToSpeakerPhoton(Photon photon, CommandSwerveDrivetrain drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.photon = photon;
    this.drive = drive; 
    addRequirements(photon, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("RUNNING");

    double currentDriveAngle; 
    double differenceAngle; 
    
    boolean hasTarget = photon.getDirectTargetVar();
        System.out.println("Command Target"  + hasTarget);

    if (hasTarget) {
      double angleCamera = photon.getAngle(); //  TODO IS RADAIAnS? ; 
     // currentDriveAngle = drive.getRotation2d().getDegrees() ;

     // double anglff = angleCamera -currentDriveAngle;
  // TODO we should try putting in a rotation2D. maybe easier//       System.out.println("Command Target"  + currentDriveAngle);
            //  System.out.println("Command Target"  + anglff);

     // drive.driveFieldCentricFacingAngle(1, 1, currentDriveAngle, angleCamera);// TODO: TEST VELCOITY SPEED WAT THEY DO
      
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
