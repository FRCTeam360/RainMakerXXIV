// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

/** Add your docs here. */
public class CommandFactory {
    private final Climber climber;
    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final Flywheel flywheel;
    private final Linkage linkage;
    // create a constructor that will require all files from the "subsystems" folder
    public CommandFactory(Climber climber, CommandSwerveDrivetrain drivetrain, Intake intake, Flywheel flywheel, Linkage linkage) {
        this.climber = climber;
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.flywheel = flywheel;
        this.linkage = linkage;
    }

    // returns type shootInSpeaker
    public ShootInSpeaker shootInSpeaker(double linkageSetpoint, double flywheelSetpoint, double driveSetpoint) {
        return new ShootInSpeaker(linkage, flywheel, drivetrain, intake, linkageSetpoint, flywheelSetpoint, driveSetpoint);
    }

    public SetClimbers setClimberShouldntFinish(double height) {
        return new SetClimbers(climber, height);
    }

    public SetClimbers setClimberShouldFinish(double height) {
        return new SetClimbers(climber, height, true);
    }

    public ShootInSpeaker shootInSpeakerWithoutDriveTrain(double linkageSetpoint, double flywheelSetpoint) {
        return new ShootInSpeaker(linkage, flywheel, intake, linkageSetpoint, flywheelSetpoint);
    }

    // returns type shootInSpeaker
    public ShootInSpeaker shootInSpeaker(double linkageSetpoint, double flywheelSetpoint) {
        return new ShootInSpeaker(linkage, flywheel, intake, linkageSetpoint, flywheelSetpoint);
    }

    public ShootInSpeaker shootFromSubwoofer() {
        return new ShootInSpeaker(linkage, flywheel, intake, 177, 6000);
    } 

    public ShootInSpeaker shootFromFar() {
        return new ShootInSpeaker(linkage, flywheel, drivetrain, intake,  157.0, 7000, 20.0);
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
        return new PowerLinkage(linkage);
    }

    // returns type runExtendIntake
    public RunExtendIntake runExtendIntake() {
        return new RunExtendIntake(intake, linkage);
    }

    // //returns type powerCenterNote
    // public PowerCenterNote powerCenterNote(){
    //     return new PowerCenterNote(intake, linkage);
    // }

    // returns type runLinkage
    // public RunLinkage runLinkage() {
    //     return new RunLinkage(linkage);
    // }

    // public SetLinkage setLinkage(double setPoint) {
    //     return new SetLinkage(linkage, setPoint);
    // }

    public SetLinkage stowLinkage() {
         return new SetLinkage(linkage, 130.0);
    }

    // returns type setIntake
    public SetIntake setIntake() {
        return new SetIntake(intake);
    }

    // public AmpArmStop ampArmStop() {
    //     return new AmpArmStop(ampArm, linkage);
    // }
}
