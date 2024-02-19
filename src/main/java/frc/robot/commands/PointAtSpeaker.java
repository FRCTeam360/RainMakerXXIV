// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PointAtSpeaker extends Command {

  public static enum TargetType {
    SPEAKER,
    STAGE
  }

  private TargetType targetType;

  private Supplier<Double> xSupplier;
  private Supplier<Double> ySupplier;
  private Supplier<Double> rotationSupplier;

  private CommandSwerveDrivetrain drivetrain;

  /** Creates a new PointAtSpeaker. */
  public PointAtSpeaker(CommandSwerveDrivetrain drivetrain, TargetType targetType, Supplier<Double> xSupplier,
      Supplier<Double> ySupplier, Supplier<Double> rotationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.targetType = targetType;

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;

    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // select the correct target based on alliance and target type
    DriverStation.Alliance alliance = DriverStation.getAlliance().get();
    // initialize target number to 0
    int targetNumber = 0;
    if (alliance == DriverStation.Alliance.Blue) {
      if (targetType == TargetType.SPEAKER) {
        // target is the speaker
        targetNumber = 7;
      } else {
        // target is the stage
        targetNumber = 14;
      }
    } else {
      if (targetType == TargetType.SPEAKER) {
        // target is the speaker
        targetNumber = 4;
      } else {
        // target is the stage
        targetNumber = 13;
      }
    }
    Camera camera = photonVision.getShooterCamera();
    if (camera.isTargetInView(targetNumber)){
      // target yaw should be retrieved from photon vision every cycle
      double targetYaw = camera.getTarget(targetNumber).getYaw();
      drivetrain.pointAtTarget(xSupplier.get(), ySupplier.get(), targetYaw);
    } else {
      // no target in view, so drive as normal
      drivetrain.fieldCentricDrive(xSupplier.get(), ySupplier.get(), rotationSupplier.get());
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
