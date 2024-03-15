// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.*;

/** Add your docs here. */
public class CommandFactory {
    private final Climber climber;
    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final Flywheel flywheel;
    private final Linkage linkage;
    private final AmpArm ampArm;
    private final Vision vision;
    private final Lights lights;
    // create a constructor that will require all files from the "subsystems" folder
    public CommandFactory(Climber climber, CommandSwerveDrivetrain drivetrain, Intake intake, Flywheel flywheel, Linkage linkage, AmpArm ampArm, Vision vision, Lights lights) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.flywheel = flywheel;
        this.linkage = linkage;
        this.ampArm = ampArm;
        this.vision = vision;
        this.lights = lights;
    }

    // returns type shootInSpeaker
    public ShootInSpeaker shootInSpeaker(double linkageSetpoint, double flywheelSetpoint, double driveSetpoint) {
        return new ShootInSpeaker(ampArm, linkage, flywheel, drivetrain, intake, linkageSetpoint, flywheelSetpoint, driveSetpoint);
    }

    public SetClimbers setClimberShouldntFinish(double height) {
        return new SetClimbers(climber, height);
    }

    public SetClimbers setClimberShouldFinish(double height) {
        return new SetClimbers(climber, height, true);
    }

    public ShootInSpeaker shootInSpeakerWithoutDriveTrain(double linkageSetpoint, double flywheelSetpoint) {
        return new ShootInSpeaker(ampArm, linkage, flywheel, intake, linkageSetpoint, flywheelSetpoint);
    }

    // returns type shootInSpeaker
    public ShootInSpeaker shootInSpeaker(double linkageSetpoint, double flywheelSetpoint) {
        return new ShootInSpeaker(ampArm, linkage, flywheel, intake, linkageSetpoint, flywheelSetpoint);
    }

    public ShootInSpeaker shootFromSubwoofer() {
        return new ShootInSpeaker(ampArm, linkage, flywheel, intake, 177, 5000);
    } 

    public ShootInSpeaker shootFromPodium() {
        return new ShootInSpeaker(ampArm, linkage, flywheel, drivetrain, intake,  0.0, 0.0, 20.0);
    }
    // returns type powerFlywheel
    public PowerFlywheel powerFlywheel() {
        return new PowerFlywheel(flywheel);
    }

    // returns type powerIntake
    public PowerIntake powerIntake() {
        return new PowerIntake(intake, lights);
    }

    // returns type powerIntakeReversed
    public PowerIntakeReversed powerIntakeReversed() {
        return new PowerIntakeReversed(intake, lights);
    }

    // returns type powerLinkage
    public PowerLinkage powerLinkage() {
        return new PowerLinkage(linkage, ampArm);
    }

    // returns type runExtendIntake
    public RunExtendIntake runExtendIntake() {
        return new RunExtendIntake(intake, linkage, ampArm);
    }

    //returns type powerCenterNote
    public PowerCenterNote powerCenterNote(){
        return new PowerCenterNote(intake, linkage, ampArm);
    }

    // returns type runLinkage
    public RunLinkage runLinkage() {
        return new RunLinkage(linkage, ampArm);
    }

    public SetLinkage setLinkage(double setPoint) {
        return new SetLinkage(linkage, setPoint, ampArm);
    }

    public SetLinkage stowLinkage() {
        return new SetLinkage(linkage, 130.0, ampArm);
    }

    public SetLinkage deploy() {
        return new SetLinkage(linkage, 0.0, ampArm);
    }

    // returns type setIntake
    public SetIntake setIntake() {
        return new SetIntake(intake);
    }

    public AmpArmStop ampArmStop() {
        return new AmpArmStop(ampArm, linkage);
    }
    public Command shootAtSpeakerUsingVision() {
        return new ParallelRaceGroup(
        new SetLinkage(linkage, 0, ampArm, vision),
        new SetFlywheel(flywheel, 0, vision),
        new PointDrivebaseAtTarget(drivetrain, vision, lights),
        new EndWhenShooterReady(linkage, flywheel, drivetrain)
        ).andThen(new ParallelRaceGroup(
            new SetLinkage(linkage, 0, ampArm, vision),
            new SetFlywheel(flywheel, 0, vision),
            new PointDrivebaseAtTarget(drivetrain, vision, lights),
            new RunCommand(()-> intake.run(1.0), intake),
            new WaitUntilCommand(0.5)
        ));
    }
}
