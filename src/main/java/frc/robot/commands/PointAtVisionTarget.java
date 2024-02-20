// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Photon;
import frc.robot.subsystems.Photon.TargetType;

public class PointAtVisionTarget extends Command {


  private TargetType targetType;

  private Supplier<Double> xSupplier;
  private Supplier<Double> ySupplier;
  private Supplier<Double> rotationSupplier;

  private CommandSwerveDrivetrain drivetrain;
  private Photon photon;

  /** Creates a new PointAtSpeaker. */
  public PointAtVisionTarget(CommandSwerveDrivetrain drivetrain, Photon photon, TargetType targetType, Supplier<Double> xSupplier,
      Supplier<Double> ySupplier, Supplier<Double> rotationSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.targetType = targetType;

    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;

    this.drivetrain = drivetrain;
    this.photon = photon;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (photon.isTargetInView(targetType)){
      // target yaw should be retrieved from photon vision every cycle
      double targetYaw = photon.getSpecifiedTargetYaw(targetType);
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
