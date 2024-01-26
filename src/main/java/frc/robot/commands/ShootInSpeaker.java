// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Linkage;
import frc.robot.subsystems.Shooter;

public class ShootInSpeaker extends Command {
  private Linkage linkage;
  private Shooter flywheel;
  private CommandSwerveDrivetrain drivetrain;
  private double linkageSetpoint;
  private double flywheelSetpoint;

  /** Creates a new ShootInSpeaker. */
  public ShootInSpeaker(Linkage linkage, Shooter flywheel,
      CommandSwerveDrivetrain drivetrain, double linkageSetpoint, double flywheelSetpoint) { // Add your commands in the
    // addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(linkage, flywheel);

    this.linkage = linkage;
    this.flywheel = flywheel;
    this.drivetrain = drivetrain;
    this.linkageSetpoint = linkageSetpoint;
    this.flywheelSetpoint = flywheelSetpoint;

  }
  @Override
  public void initialize() {
    state = ShootState.LOADED;
  }

  @Override
  public void execute() {
    switch (state) {
      case LOADED:
      linkage.setAngle(linkageSetpoint);
      boolean isLinkageAtSetpoint = linkage.isAtSetpoint();
      boolean isFlywheelAtSetpoint = flywheel.isAtSetpoint();
      boolean isDrivetrainAtSetpoint = drivetrain.isFacingAngle();
      if(isLinkageAtSetpoint){
        System.out.println("inkage at setpoint");
      }
      if(isFlywheelAtSetpoint){
        System.out.println("flywheel at setpoint");
      }
      if(isDrivetrainAtSetpoint){
        System.out.println("drivetrain at setpoint");
      }
      if(isLinkageAtSetpoint && isDrivetrainAtSetpoint && isFlywheelAtSetpoint){
       this.state = ShootState.SHOOT;
      }
      break;
      
      

    }

  }

  private ShootState state = ShootState.LOADED;

  private enum ShootState {
    LOADED, SHOOT
  }

  public class ShootInSpeakerParallel extends ParallelCommandGroup {
    public ShootInSpeakerParallel(CommandSwerveDrivetrain drivetrain, Linkage linkage, Shooter flywheel,
        double flywheelSetpoint, double linkageSetpoint, double driveAngleSetpoint) {

      // convert the drive angle setpoint to a rotation2d
      Rotation2d driveRotSetpoint = new Rotation2d(driveAngleSetpoint);
      // add the commands to the parallel command group
      addCommands(new ShootInSpeaker(linkage, flywheel, drivetrain, linkageSetpoint, flywheelSetpoint),
          drivetrain.turntoCMD(driveRotSetpoint, flywheelSetpoint, driveAngleSetpoint));
    }
  }

}
