// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;

public class ShootInSpeaker extends Command {
  private final Linkage linkage;
  private final Flywheel flywheel;
  private final CommandSwerveDrivetrain drivetrain;
  private final AmpArm arm;

  private double linkageSetpoint;
  private double flywheelSetpoint;
  private double driveAngleSetpoint;

  private Timer timer = new Timer();
  private Intake intake;

  private ShootState state = ShootState.LOADED;

  private enum ShootState {
    LOADED, SHOOT, TIMER, END
  }

  /** Creates a new ShootInSpeaker. */
  public ShootInSpeaker(AmpArm ampArm, Linkage linkage, Flywheel flywheel,
      CommandSwerveDrivetrain drivetrain, Intake intake,
      double linkageSetpoint, double flywheelSetpoint, double driveSetpoint) { // Add your commands in the
    // addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(linkage, flywheel, intake, drivetrain);

    this.linkage = linkage;
    this.flywheel = flywheel;
    this.drivetrain = drivetrain;
    this.linkageSetpoint = linkageSetpoint;
    this.flywheelSetpoint = flywheelSetpoint;
    this.driveAngleSetpoint = driveSetpoint;
    this.intake = intake;
    this.arm = ampArm;

    // withDriveTrain = true;
  }

  @Override
  public void initialize() {
    state = ShootState.LOADED;
    timer.stop();
    timer.reset();
  }

  public ShootInSpeaker(AmpArm ampArm, Linkage linkage, Flywheel flywheel, Intake intake,
      double linkageSetpoint, double flywheelSetpoint) { // Add your commands in the
    // addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addRequirements(linkage, flywheel, intake);

    this.linkage = linkage;
    this.flywheel = flywheel;
    this.linkageSetpoint = linkageSetpoint;
    this.flywheelSetpoint = flywheelSetpoint;
    this.intake = intake;
    this.arm = ampArm;
    this.drivetrain = null;

    // withDriveTrain = false;
  }

  


  @Override
  public void execute() {
    System.out.println("SHOOTING SHOOTNIG SHOOTING");
    if (!Objects.isNull(drivetrain)) {
      drivetrain.driveFieldCentricFacingAngle(0.0, 0.0, 0.0, driveAngleSetpoint); // drivetrain is rotated in its own
                                                                                  // command ran in // parallel
    } 
    linkage.setAngle(linkageSetpoint, arm);
    System.out.println("this is the robot state: " + this.state);
    flywheel.setBothRPM(flywheelSetpoint);
    System.out.println("left velocity: " + flywheel.getLeftVelocity());
    System.out.println("is above setpoint " + flywheel.isAboveSetpoint());
    switch (state) {
      case LOADED:
        intake.stop();
        boolean isLinkageAtSetpoint = linkage.isAtSetpoint();
        boolean isFlywheelAtSetpoint = flywheel.isAtSetpoint();
     //   boolean isDriveReady = Objects.isNull(drivetrain) || drivetrain.isFacingAngle();
        if (isFlywheelAtSetpoint && isLinkageAtSetpoint) { // && isLinkageAtSetpoint
          this.state = ShootState.SHOOT;
        }
        break;

      case SHOOT:
        intake.run(1.0);
        boolean hasShot = flywheel.isBelowSetpoint(); // check logic in flywheel subsystem (180 rpm gap)
        if (hasShot) {
          timer.start();
          state = ShootState.TIMER;
          System.out.println(state);
          // timer.start();
          // if (timer.hasElapsed(0.3)) { // TUNE!!!
          // this.state = ShootState.END;
        }
        break;
      case TIMER:
        intake.run(1.0);
        if (timer.hasElapsed(0.3) && intake.getSideSensor() && intake.getDiagonalSensor()) {
          this.state = ShootState.END;
        }
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    flywheel.stop();
    // linkage.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (state == ShootState.END) {
      return true;
    } else {
      return false;
    }
  }
}
