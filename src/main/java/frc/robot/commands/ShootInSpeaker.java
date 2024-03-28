// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Driver;
import java.util.Objects;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.UtilMethods;
import frc.robot.utils.CommandLogger;

public class ShootInSpeaker extends Command {
  private final Linkage linkage;
  private final Flywheel flywheel;
  private final CommandSwerveDrivetrain drivetrain;
  private final AmpArm arm;

  private double linkageSetpoint;
  private double flywheelSetpoint;
  private double driveAngleSetpoint;

  private final XboxController driverController = new XboxController(0);

  private Timer timer = new Timer();
  private Timer loadedtimer = new Timer();
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
    CommandLogger.logCommandStart(this);
    state = ShootState.LOADED;
    timer.stop();
    timer.reset();
    loadedtimer.stop();
    loadedtimer.reset();
    loadedtimer.start();
    if(!intake.hasNote()){
      state = ShootState.END;
    }
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

  public double getWithDeadband(double input) {
    if (Math.abs(input) < 0.1) {
      input = 0.0;
    }
    return input;
  }
  
  @Override
  public void execute() {
    // System.out.println("SHOOTING SHOOTNIG SHOOTING");
    if (!Objects.isNull(drivetrain)) {
      driveAngleSetpoint = DriverStation.getAlliance().get() == Alliance.Red ? 20.0 : -20.0;
      //drivetrain.turntoCMD(false,  UtilMethods.squareInput(getWithDeadband(driverController.getLeftX())),  UtilMethods.squareInput(getWithDeadband(driverController.getLeftY())), driveAngleSetpoint);
      drivetrain.driveFieldCentricFacingAngle(UtilMethods.squareInput(getWithDeadband(-driverController.getLeftY())), UtilMethods.squareInput(getWithDeadband(-driverController.getLeftX())), driveAngleSetpoint); // drivetrain is rotated in its own
                                                                                  // command ran in // parallel
    } 
    linkage.setAngle(linkageSetpoint, arm);
    // System.out.println("this is the robot state: " + this.state);
    Logger.recordOutput("ShootInSpeaker: State", this.state);
    flywheel.setBothRPM(flywheelSetpoint);
    // System.out.println("left velocity: " + flywheel.getLeftVelocity());
    // System.out.println("is above setpoint " + flywheel.isAtSetpoint());
    // System.out.println("linkage is at SETPOINT" + linkage.isAtSetpoint());
    switch (state) {
      case LOADED:
        intake.stop();
        boolean isLinkageAtSetpoint = linkage.isAtSetpoint();
        boolean isFlywheelAtSetpoint = flywheel.isAboveSetpoint();
        Logger.recordOutput("ShootInSpeaker: Linkage Setpoint", isLinkageAtSetpoint);
        Logger.recordOutput("ShootInSpeaker: Flywheel Setpoint", isFlywheelAtSetpoint);
     //   boolean isDriveReady = Objects.isNull(drivetrain) || drivetrain.isFacingAngle();
        if (isFlywheelAtSetpoint && (loadedtimer.get() > 0.3 || isLinkageAtSetpoint)) { // && isLinkageAtSetpoint
          this.state = ShootState.SHOOT;
        }
        break;

      case SHOOT:
        intake.run(1.0);
        boolean hasShot = flywheel.isBelowSetpoint(); // check logic in flywheel subsystem (180 rpm gap)
        Logger.recordOutput("ShootInSpeaker: hasShot", hasShot);
        if (hasShot) {
          timer.start();
          state = ShootState.TIMER;
          // System.out.println(state);
          // timer.start();
          // if (timer.hasElapsed(0.3)) { // TUNE!!!
          // this.state = ShootState.END;
        }
        break;
      case TIMER:
        intake.run(1.0);
        if (timer.hasElapsed(0.3) && intake.getSideSensor() && intake.getShooterSensor()) {
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
    CommandLogger.logCommandEnd(this);
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
