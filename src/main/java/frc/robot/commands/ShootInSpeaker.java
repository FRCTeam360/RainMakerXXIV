// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/** 

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Linkage;

public class ShootInSpeaker extends Command {
  private Linkage linkage;
  private Flywheel flywheel;
  private CommandSwerveDrivetrain drivetrain;

  private double linkageSetpoint;
  private double flywheelSetpoint;
  private Timer timer;

  private ShootState state = ShootState.LOADED;

  private enum ShootState {
    LOADED, SHOOT
  }

  /** Creates a new ShootInSpeaker. 
  public ShootInSpeaker(Linkage linkage, Flywheel flywheel,
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
    linkage.setAngle(linkageSetpoint);
    flywheel.setBothRPM(flywheelSetpoint);
    // drivetrain is rotated in its own command ran in parallel
    switch (state) {
      case LOADED:
        boolean isLinkageAtSetpoint = linkage.isAtSetpoint();
        boolean isFlywheelAtSetpoint = flywheel.isAtSetpoint();
        boolean isDrivetrainAtSetpoint = drivetrain.isFacingAngle();
        if (isLinkageAtSetpoint) {
          System.out.println("inkage at setpoint");
        }
        if (isFlywheelAtSetpoint) {
          System.out.println("flywheel at setpoint");
        }
        if (isDrivetrainAtSetpoint) {
          System.out.println("drivetrain at setpoint");
        }
        if (isLinkageAtSetpoint && isDrivetrainAtSetpoint && isFlywheelAtSetpoint) {
          this.state = ShootState.SHOOT;
        }
        break;

      case SHOOT:
        boolean hasShot = flywheel.isBelowSetpoint(); //check logic in flywheel subsystem (180 rpm gap)
        if (hasShot) {
          timer.start();
          if (timer.hasElapsed(0.2)) { //TUNE!!!
            isFinished(); //sketch
          }
        }
        break;

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  public class ShootInSpeakerParallel extends ParallelCommandGroup {
    public ShootInSpeakerParallel(CommandSwerveDrivetrain drivetrain, Linkage linkage, Flywheel flywheel,
        double flywheelSetpoint, double linkageSetpoint, double driveAngleSetpoint) {

      // convert the drive angle setpoint to a rotation2d
      Rotation2d driveRotSetpoint = new Rotation2d(driveAngleSetpoint);
      // add the commands to the parallel command group
      addCommands(new ShootInSpeaker(linkage, flywheel, drivetrain, linkageSetpoint, flywheelSetpoint),
          drivetrain.turntoCMD(driveRotSetpoint, flywheelSetpoint, driveAngleSetpoint));
    }
  }

}
*/