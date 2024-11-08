// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.subsystems.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.*;

/** Add your docs here. */
public class CommandFactory {
    private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);
    private final Climber climber;
    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final Flywheel flywheel;
    private final Linkage linkage;
    private final AmpArm ampArm;
    private final Vision vision;

    // create a constructor that will require all files from the "subsystems" folder
    public CommandFactory(Climber climber, CommandSwerveDrivetrain drivetrain, Intake intake, Flywheel flywheel,
            Linkage linkage, AmpArm ampArm, Vision vision) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.flywheel = flywheel;
        this.linkage = linkage;
        this.ampArm = ampArm;
        this.vision = vision;
    }

    // returns type shootInSpeaker
    public ShootInSpeaker shootInSpeaker(double linkageSetpoint, double flywheelSetpoint, double driveSetpoint) {
        return new ShootInSpeaker(ampArm, linkage, flywheel, drivetrain, intake, linkageSetpoint, flywheelSetpoint,
                driveSetpoint);
    }

    public SetClimbers setClimberShouldntFinish(double height) {
        return new SetClimbers(climber, height);
    }

    private class EndWhenShot extends Command {
        private Intake intake;
        private Flywheel flywheel;
        private Timer timer = new Timer();

        public EndWhenShot(Intake intake, Flywheel flywheel) {
            this.intake = intake;
            this.flywheel = flywheel;
        }

        public void initialize() {
            CommandLogger.logCommandStart(this);
            timer.reset();
            timer.start();
        }

        public void end(boolean isInterrupted) {
            timer.stop();
            CommandLogger.logCommandEnd(this);
        }

        public boolean isFinished() {
            Logger.recordOutput("end when shot: shooter sensor", intake.getShooterSensor());
            Logger.recordOutput("end when shot: side sensor", intake.getSideSensor());

            Logger.recordOutput("end when shot: timer", timer.get());

            return timer.hasElapsed(.3) && intake.getSideSensor() && intake.getShooterSensor();
        }

    }

    public SetClimbers setClimberShouldFinish(double height) {
        return new SetClimbers(climber, height, true);
    }

    public ShootInSpeaker shootInSpeakerWithoutDriveTrain(double linkageSetpoint, double flywheelSetpoint) {
        return new ShootInSpeaker(ampArm, linkage, flywheel, intake, linkageSetpoint, flywheelSetpoint);
    }

    private final double SUBWOOFER_LINKAGE = 177.0;
    private final double SUBWOOFER_FLYWHEEL = 5000.0;

    // returns type shootInSpeaker
    public ShootInSpeaker shootInSpeaker(double linkageSetpoint, double flywheelSetpoint) {
        return new ShootInSpeaker(ampArm, linkage, flywheel, intake, linkageSetpoint, flywheelSetpoint);
    }

    public ShootInSpeaker shootFromSubwoofer() {
        return new ShootInSpeaker(ampArm, linkage, flywheel, intake, 177, 5000);
    }

    public Command shootFromSubwooferSpinUp() {
        return new ParallelCommandGroup(
                new SetLinkage(linkage, SUBWOOFER_LINKAGE, ampArm, false),
                new SetFlywheel(flywheel, SUBWOOFER_FLYWHEEL));
    }

    public PointDrivebaseAtTarget pointDriveBaseAtTarget() {
        return new PointDrivebaseAtTarget(drivetrain, vision);
    }

    public ShootInSpeaker shootFromPodium() {
        return new ShootInSpeaker(ampArm, linkage, flywheel, drivetrain, intake, 0.0, 0.0, 20.0);
    }

    // returns type powerFlywheel
    public PowerFlywheel powerFlywheel() {
        return new PowerFlywheel(flywheel);
    }

    // returns type powerIntake
    public PowerIntake powerIntake() {
        return new PowerIntake(intake);
    }

    // returns type powerIntakeReversed
    public PowerIntakeReversed powerIntakeReversed() {
        return new PowerIntakeReversed(intake);
    }

    // returns type powerLinkage
    public PowerLinkage powerLinkage() {
        return new PowerLinkage(linkage, ampArm);
    }

    // returns type runExtendIntake
    public RunExtendIntake runExtendIntake() {
        return new RunExtendIntake(intake, linkage, ampArm);
    }

    // returns type powerCenterNote
    public PowerCenterNote powerCenterNote() {
        return new PowerCenterNote(intake, linkage, ampArm);
    }

    // returns type runLinkage
    public RunLinkage runLinkage() {
        return new RunLinkage(linkage, ampArm);
    }

    public SetLinkage setLinkage(double setPoint, boolean shouldEnd) {
        return new SetLinkage(linkage, setPoint, ampArm, shouldEnd);
    }

    public Command tuneLinkageSetpoint() {
        return new ParallelCommandGroup(
                new LinkageSetpoint(linkage, ampArm),
                new SetFlywheel(flywheel, 7500));
    }

    public SetLinkage stowLinkage() {
        return new SetLinkage(linkage, 140.0, ampArm, false); // was 130
    }

    public SetLinkage deploy() {
        return new SetLinkage(linkage, 0.0, ampArm, false);
    }

    // returns type setIntake
    public SetIntake setIntake() {
        return new SetIntake(intake);
    }

    public AmpArmStop ampArmStop() {
        return new AmpArmStop(ampArm, linkage);
    }

    public Command shootAtSpeakerVision() {
        return new ParallelRaceGroup(
                spinUpSpeakerVision(),
                new EndWhenShooterReady(linkage, flywheel, drivetrain, vision)
                        .andThen(
                                new ParallelRaceGroup(
                                        new RunCommand(() -> intake.run(1.0), intake),
                                        new EndWhenShot(intake, flywheel)))).andThen(takeSnapshot());
    }

    public Command shootAtSpeakerVisionAuto() {
        return shootAtSpeakerVision().raceWith(new InterruptWhenNoNote(intake));
    }

    private class InterruptWhenNoNote extends Command {
        boolean shouldEnd = false;
        private final Intake intake;

        @Override
        public void initialize() {
            shouldEnd = !intake.hasNote();
        }

        @Override
        public boolean isFinished() {
            return shouldEnd;
        }

        public InterruptWhenNoNote(Intake intake) {
            this.intake = intake;
        }
    }

    public Command spinUpSpeakerVision() {
        Command pointDrivebaseAtTarget = new PointDrivebaseAtTarget(drivetrain, vision);
        return new ParallelCommandGroup(
                new SetLinkage(linkage, 170, ampArm, vision, false),
                new SetFlywheelVision(flywheel, 7500),
                pointDrivebaseAtTarget);
    }

    public Command spinUpShooterSetpoint(double linkageSetpoint, double flywheelSetpoint) {
        return new ParallelCommandGroup(
                new SetLinkage(linkage, linkageSetpoint, ampArm, false),
                new SetFlywheel(flywheel, flywheelSetpoint));
    }

    public Command spinUpShooterSetpointWithDrivebase(double linkageSetpoint, double flywheelSetpoint) {
        return new ParallelCommandGroup(
                spinUpShooterSetpoint(linkageSetpoint, flywheelSetpoint),
                new PointDrivebaseAtTarget(drivetrain, vision));
    }

    public Command takeSnapshot() {
        return new TakeSnapshot(vision);
    }

    public Command spinUpForOverPass() {
        return new ParallelCommandGroup(
                spinUpShooterSetpoint(170.0, 5000.0 * 0.85),
                new DriveFieldCentricFacingAngle(drivetrain, -150.0, -40.0));
    }

    public Command spinUpForOverPassAndShoot() {
        return new ParallelCommandGroup(
                spinUpForOverPass(),
                new EndWhenShooterReady(linkage, flywheel, drivetrain, vision)
                        .andThen(
                                new RunCommand(() -> intake.run(1.0), intake)));
    }

    public Command spinUpForUnderPass() {
        return new ParallelCommandGroup(
                spinUpShooterSetpoint(82.0, 4500.0 * 0.85),
                new DriveFieldCentricFacingAngle(drivetrain, -140.0, -40.0));
    }

    public Command spinUpForUnderPassAndShoot() {
        return new ParallelCommandGroup(
                spinUpForUnderPass(),
                new EndWhenShooterReady(linkage, flywheel, drivetrain, vision)
                        .andThen(
                                new RunCommand(() -> intake.run(1.0), intake)));
    }

    public Command rumbleDriverController(CommandXboxController controller) {
        return Commands.runEnd(() -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0),
                            () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0));
    }

    private class TakeSnapshot extends InstantCommand {
        public TakeSnapshot(Vision vision) {
            super(() -> vision.takeSnapshot());
        }
    }
}
