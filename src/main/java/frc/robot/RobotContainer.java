// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotType;
import frc.robot.commands.DiagonalSensorIntake;
import frc.robot.commands.RunExtendIntake;
import frc.robot.commands.SetClimbers;
import frc.robot.commands.ScoreInAmp;
import frc.robot.commands.PowerIntakeReversed;
import frc.robot.commands.PowerIntake;
import frc.robot.commands.PowerLinkage;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SetLinkage;
import frc.robot.commands.ShootInSpeaker;
import frc.robot.commands.TrapClimb;
import frc.robot.commands.TrapSetUp;
import frc.robot.commands.TuneFlywheel;
import frc.robot.commands.TuneSwerveDrive;
import frc.robot.commands.PowerFlywheel;
import frc.robot.commands.RobotOrientedDrive;
import frc.robot.commands.AmpArmGoToZero;
import frc.robot.commands.AmpArmNote;
import frc.robot.commands.AmpArmStop;
import frc.robot.commands.AutoPowerCenterNote;
import frc.robot.commands.BasicClimb;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.HomeAmpArmWrist;
import frc.robot.commands.IntakeCOmmand;
import frc.robot.commands.LevelClimbers;
import frc.robot.commands.ClimberPIDTuner;
import frc.robot.commands.LinkageSetpoint;
import frc.robot.commands.LinkageToAmpHandoff;
import frc.robot.commands.PowerAmpArm;
import frc.robot.commands.PowerAmpIntake;
import frc.robot.commands.PowerAmpIntakeReverse;
import frc.robot.commands.PowerCenterNote;
import frc.robot.commands.PowerClimber;
import frc.robot.generated.CompBotConstants;
import frc.robot.generated.PracticebotConstants;
import frc.robot.generated.WoodbotConstants;
import frc.robot.hardware.AmpArmIOTalonFX;
import frc.robot.hardware.AmpIntakeIOSparkMax;
import frc.robot.hardware.ClimberIOSparkMax;
import frc.robot.hardware.FlywheelIOSparkFlex;
import frc.robot.hardware.IntakeIOSparkFlex;
import frc.robot.hardware.IntakeIOSparkMax;
import frc.robot.hardware.LinkageIOTalonFX;
import frc.robot.hardware.LinkageIOSparkMax;
import frc.robot.io.FlywheelIO;
import frc.robot.io.IntakeIO;
import frc.robot.sim.ShooterIOSim;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.AmpIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Linkage;
import frc.robot.utils.CommandFactory;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.Objects;
import java.util.Optional;

import javax.management.InstanceNotFoundException;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // declared as final in example code, but gives error in our code
  private SendableChooser<Command> autoChooser;

  private final CommandXboxController operatorController = new CommandXboxController(Constants.OPERATOR_CONTROLLER);
  private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);

  final double MAX_SPEED_MPS = Constants.MAX_SPEED_MPS; // used to be 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI * 3; // Half a rotation per second max angular velocity
  // dio ports
  private DigitalInput zeroButton = new DigitalInput(Constants.LINKAGE_ZERO_BUTTON_PORT);
  private DigitalInput brakeButton = new DigitalInput(Constants.LINKAGE_BRAKE_TOGGLE_BUTTON_PORT);

  // subsystems
  private CommandSwerveDrivetrain drivetrain; // My drivetrain
  private Flywheel flywheel;
  private Linkage linkage;
  private Intake intake;
  private Climber climber;
  private AmpArm ampArm;
  private AmpIntake ampIntake;
  private CommandFactory commandFactory;

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private ShootInSpeaker shootRoutine;
  private RunExtendIntake runExtendIntake;
  private DiagonalSensorIntake diagonalSensorIntakeCloseShot;

  private PowerCenterNote powerCenterNoteIntakeRoutine;
  private PowerIntakeReversed powerIntakeReversed;
  private PowerIntake powerIntake;
  private PowerFlywheel powerFlywheel;
  private PowerClimber powerClimber;
  private PowerAmpArm powerAmpArm;
  private PowerAmpIntake powerAmpIntake;
  private PowerLinkage powerLinkage;

  private ShuffleboardTab diagnosticTab;
  private FieldOrientedDrive fieldOrientedDrive;
  private RobotOrientedDrive robotOrientedDrive;
  private ClimberPIDTuner pidTuner;
  private SetClimbers maxExtend;
  private SetClimbers minExtend;

  private IntakeCOmmand longerinny;
  // private SetLinkageTalon setLinkageTalon = new SetLinkageTalon(linkage);
  private SetLinkage setLinkage;
  private IntakeCOmmand ryryinny;
  private SetLinkage stowLinkage;
  private LinkageSetpoint linkageSetpoint;
  private TuneFlywheel tuneFlywheel;
  private ShootInSpeaker shootFromSubwoofer;
  private ShootInSpeaker shootFromFar;
  private TuneSwerveDrive tuneSwerveDrive;
  private AutoPowerCenterNote autoPowerCenterNote;
  private PowerAmpIntakeReverse powerAmpIntakeReverse;
  private AmpArmNote ampArmNote;
  private IntakeCOmmand inny;
  private ScoreInAmp scoreInAmp;
  private LinkageToAmpHandoff linkageToAmpHandoff;
  private AmpArmStop ampArmStop;
  private BasicClimb basicClimb;
  private ShootInSpeaker passUnderStage;

  private SetClimbers goToZero;
  private SetClimbers fullRetract;
  private SetClimbers soloClimb;

  private SetLinkage deploy;

  private HomeAmpArmWrist homeAmpArmWrist;
  private AmpArmGoToZero ampArmGoToZero;

  private TrapSetUp trapDrive;
  private TrapClimb trapClimb; 

  final Rotation2d setAngle = Rotation2d.fromDegrees(0);

  /* Setting up bindings for necessary control of the swerve drive platform */
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(Constants.MAX_SPEED_MPS);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.getRobotType()) {
      case WOODBOT:
        // Real robot, instantiate hardware IO implementations
        flywheel = new Flywheel(new FlywheelIOSparkFlex());
        intake = new Intake(new IntakeIOSparkMax());
        linkage = new Linkage(new LinkageIOSparkMax());
        drivetrain = WoodbotConstants.DriveTrain;
        // ampArm = new AmpArm(new AmpArmIOTalonFX());
        // ampIntake = new AmpIntake(new AmpIntakeIOSparkMax());
        break;
      case PRACTICE:
        flywheel = new Flywheel(new FlywheelIOSparkFlex());
        intake = new Intake(new IntakeIOSparkFlex());
        linkage = new Linkage(new LinkageIOTalonFX(zeroButton, brakeButton));
        drivetrain = PracticebotConstants.DriveTrain; // My drivetrain
        climber = new Climber(new ClimberIOSparkMax());
        // ampArm = new AmpArm(new AmpArmIOTalonFX());
        // ampIntake = new AmpIntake(new AmpIntakeIOSparkMax());

        drivetrain = PracticebotConstants.DriveTrain; // My drivetrain
        drivetrain.configNeutralMode(NeutralModeValue.Coast);
        break;
      case COMPETITION:
        drivetrain = CompBotConstants.DriveTrain;
        flywheel = new Flywheel(new FlywheelIOSparkFlex());
        intake = new Intake(new IntakeIOSparkFlex());
        ampArm = new AmpArm(new AmpArmIOTalonFX(zeroButton, brakeButton));
        ampIntake = new AmpIntake(new AmpIntakeIOSparkMax());
        climber = new Climber(new ClimberIOSparkMax());
        linkage = new Linkage(new LinkageIOTalonFX(zeroButton, brakeButton));
        break;
      case TEST:

        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // CHANGE SHOOTER AND SHOOTER LINKAGE TO SIM LATER
        // shooter = new Shooter(new ShooterIOSparkMax());
        // intake = new Intake(new IntakeIOSparkMax());
        // linkage = new Linkage(new LinkageIOSparkMax());
        // commandSwerveDrivetrain = new CommandSwerveDrivetrain(new
        // CommandSwerveDrivetrainIOSparkMax());
        break;
      case REPLAY:

        break;

      default:
        // Replayed robot, disable IO implementations
        // shooter = new Shooter(new ShooterIO() {});
        // intake = new Intake(new IntakeIO() {});
        // shooterLinkage = new ShooterLinkage(new ShooterLinkageIO() {});
        break;
    }
    diagnosticTab = Shuffleboard.getTab("Diagnostics");
    diagnosticTab.addBoolean("Test Bot", () -> Constants.isTestBot());
    diagnosticTab.addBoolean("Wood Bot", () -> Constants.isWoodBot());
    diagnosticTab.addBoolean("Practice Bot", () -> Constants.isPracticeBot());
    diagnosticTab.addBoolean("Comp Bot", () -> Constants.isCompBot());
    initializeCommands();

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
    configureDefaultCommands();
  }

  private final void initializeCommands() {
    if (Objects.nonNull(ampArm) && Objects.nonNull(ampIntake)) {
      scoreInAmp = new ScoreInAmp(ampArm, ampIntake, linkage);
      linkageToAmpHandoff = new LinkageToAmpHandoff(linkage, ampArm, ampIntake, flywheel, intake);
      ampArmNote = new AmpArmNote(ampIntake);
    }
    diagonalSensorIntakeCloseShot = new DiagonalSensorIntake(ampArm, flywheel, intake, linkage, 6000.0);
    commandFactory = new CommandFactory(climber, drivetrain, intake, flywheel, linkage, ampArm);
    fieldOrientedDrive = new FieldOrientedDrive(drivetrain, linkage, ampArm);
    robotOrientedDrive = new RobotOrientedDrive(drivetrain);
    runExtendIntake = commandFactory.runExtendIntake();
    autoPowerCenterNote = new AutoPowerCenterNote(ampArm, intake, linkage, flywheel, 177.0);
    powerCenterNoteIntakeRoutine = commandFactory.powerCenterNote();

    powerIntakeReversed = new PowerIntakeReversed(intake);
    powerIntake = new PowerIntake(intake);
    powerFlywheel = new PowerFlywheel(flywheel);
    powerClimber = new PowerClimber(climber);
    shootRoutine = new ShootInSpeaker(ampArm, linkage, flywheel, drivetrain, intake, 0.0, 5000.0, 90.0);
    maxExtend = new SetClimbers(climber, 70.0);
    passUnderStage = new ShootInSpeaker(ampArm, linkage, flywheel, drivetrain, intake, 106.0, 4000.0,
        fetchAllianceNum());
    minExtend = new SetClimbers(climber, -35.0);
    // levelClimbers = new LevelClimbers(climber, drivetrain);
    tuneFlywheel = new TuneFlywheel(flywheel);
    linkageSetpoint = new LinkageSetpoint(linkage, ampArm);
    powerLinkage = new PowerLinkage(linkage, ampArm);
    stowLinkage = commandFactory.stowLinkage();
    powerAmpIntakeReverse = new PowerAmpIntakeReverse(ampIntake);
    inny = new IntakeCOmmand(intake, linkage, ampArm, 177.0, true);
    longerinny = new IntakeCOmmand(intake, linkage, ampArm, 144.0, true);
    ryryinny = new IntakeCOmmand(intake, linkage, ampArm, 0.0, false);
    powerLinkage = commandFactory.powerLinkage();
    shootRoutine = commandFactory.shootInSpeaker(177.0, 6000.0);
    // autoCenterNote = commandFactory.shootInSpeaker(160.0, 6000.0);
    shootFromSubwoofer = commandFactory.shootFromSubwoofer();
    shootFromFar = commandFactory.shootFromFar();
    basicClimb = new BasicClimb(climber);

    deploy = commandFactory.deploy();

    trapDrive = new TrapSetUp(drivetrain, linkage, ampArm, climber);
    trapClimb = new TrapClimb(ampArm, climber, linkage);
    
    goToZero = commandFactory.setClimberShouldFinish(0);
    soloClimb = commandFactory.setClimberShouldFinish(40);
    fullRetract = commandFactory.setClimberShouldFinish(-57);


    // COMMENT OUT tuneSwerveDrive WHEN NOT USING, IT WILL SET YOUR SWERVE DRIVE
    // CONSTANTS TO 0 WHEN CONSTRUCTED
    // tuneSwerveDrive = new TuneSwerveDrive(drivetrain);
    if (!Objects.isNull(ampArm)) {
      powerAmpArm = new PowerAmpArm(ampArm, linkage);
      ampArmStop = commandFactory.ampArmStop();
      homeAmpArmWrist = new HomeAmpArmWrist(ampArm, linkage);
      ampArmGoToZero = new AmpArmGoToZero(ampArm, linkage);
    }
    if (!Objects.isNull(ampIntake)) {
      powerAmpIntakeReverse = new PowerAmpIntakeReverse(ampIntake);
      powerAmpIntake = new PowerAmpIntake(ampIntake);
      powerAmpIntakeReverse = new PowerAmpIntakeReverse(ampIntake);
    }

    // powerAmpArm = new PowerAmpArm(ampArm);
    // powerAmpIntake = new PowerAmpIntake(ampIntake);

    Command shootRoutineWithDrivetrain = new ShootInSpeaker(ampArm, linkage, flywheel, drivetrain, intake, 0.0, 5000.0,
        0.0);
    NamedCommands.registerCommand("Intake", inny);
    NamedCommands.registerCommand("Auto Center Note", new AutoPowerCenterNote(ampArm, intake, linkage, flywheel, 163));
    NamedCommands.registerCommand("Wait1", new WaitCommand(1));
    NamedCommands.registerCommand("Shoot", shootRoutineWithDrivetrain);
    NamedCommands.registerCommand("Rotate", drivetrain.turntoCMD(false, 45.0, 0, 0));
    NamedCommands.registerCommand("Shoot without drivetrain", shootRoutine);
    NamedCommands.registerCommand("Shoot from subwoofer", shootFromSubwoofer);
    NamedCommands.registerCommand("Spinny", new PowerFlywheel(flywheel));
    NamedCommands.registerCommand("AutoShot1", new ShootInSpeaker(ampArm, linkage, flywheel, intake, 163.0, 6500.0));
    NamedCommands.registerCommand("extend linkage", new InstantCommand(() -> linkage.setAngle(0.0, ampArm), linkage));
    NamedCommands.registerCommand("linkage long prep",
        new InstantCommand(() -> linkage.setAngle(151, ampArm), linkage));
    NamedCommands.registerCommand("stay out of way shot",
        new ShootInSpeaker(ampArm, linkage, flywheel, intake, 151, 7000.0));
    NamedCommands.registerCommand("long shot inny", longerinny);
    NamedCommands.registerCommand("last guy", new ShootInSpeaker(ampArm, linkage, flywheel, intake, 153, 7000.0));
    NamedCommands.registerCommand("blue linkage long prep",
        new InstantCommand(() -> linkage.setAngle(148, ampArm), linkage));
    NamedCommands.registerCommand("blue stay out of way shot",
        new ShootInSpeaker(ampArm, linkage, flywheel, intake, 148, 7000.0));
    NamedCommands.registerCommand("blue last guy",
        new ShootInSpeaker(ampArm, linkage, flywheel, intake, 151.5, 7000.0));

    // NamedCommands.registerCommand("Intake", runExtendIntake);
    // NamedCommands.registerCommand("Wait1", new WaitCommand(1));
    // NamedCommands.registerCommand("Wait", new WaitCommand(2));
    // NamedCommands.registerCommand("Shoot", shootRoutine);
    // NamedCommands.registerCommand("Rotate", drivetrain.turntoCMD(false, 45.0, 0,
    // 0));
    // NamedCommands.registerCommand("Shoot without drivetrain", new
    // ShootInSpeaker(linkage, flywheel, drivetrain, intake, MAX_SPEED_MPS,
    // MaxAngularRate, MAX_SPEED_MPS));
    // fieldOrientedDrive = new FieldOrientedDrive();
    // robotOrientedDrive = new RobotOrientedDrive();
    // drivetrain = TunerConstants.DriveTrain; // My drivetrain
  }

  private void configureDefaultCommands() {

    // DRIVER CONTROLS DO NOT DELETE
    drivetrain.setDefaultCommand(fieldOrientedDrive);

    // OPERATOR CONTROLS DO NOT DELETE
    // climber.setDefaultCommand(powerClimber);
    intake.setDefaultCommand(powerIntake);
    // linkage.setDefaultCommand(powerLinkage);

    // linkage.setDefaultCommand(powerLinkage);
    // if (Objects.nonNull(ampArm)) {
    // ampArm.setDefaultCommand(powerAmpArm);
    // }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // DRIVER CONTROLS DO NOT DELETE JUST COMMENT OUT
    driverController.leftBumper().whileTrue(powerIntakeReversed);
    driverController.rightBumper().whileTrue(inny);

    driverController.b().whileTrue(stowLinkage);
    driverController.a().whileTrue(shootFromSubwoofer);
    driverController.y().whileTrue(passUnderStage);

    driverController.pov(180).whileTrue(new InstantCommand(() -> drivetrain.zero(), drivetrain));
    driverController.pov(0).whileTrue(deploy);
    
          

    // OPERATOR CONTROLS DO NOT DELETE JUST COMMENT OUT

    operatorController.leftBumper().whileTrue(powerAmpIntakeReverse);
    operatorController.rightBumper().whileTrue(powerAmpIntake);

    if (Objects.nonNull(ampArm)) {
      driverController.pov(90)
        .toggleOnTrue(
          trapDrive
          );
      operatorController.x().toggleOnTrue(linkageToAmpHandoff);
      // operatorController.a().onTrue(scoreInAmp);
      // operatorController.a().toggleOnTrue(new InstantCommand(() -> ampArm.setArm(108.5, linkage)));
      // operatorController.a().toggleOnTrue(new InstantCommand(() -> ampArm.setWrist(140.3)));
      operatorController.y().toggleOnTrue(new InstantCommand(() -> {
        ampArm.setArm(-6.0, linkage);
        ampArm.setWrist(45);
      }, ampArm));
      operatorController.a().toggleOnTrue(new InstantCommand(() -> ampIntake.runIntake(.5)));
      operatorController.pov(90).toggleOnTrue(trapClimb);


      // operatorController.pov(90).onTrue(homeAmpArmWrist);
      // operatorController.pov(180).onTrue(ampArmGoToZero);
    }

    operatorController.start().negate().whileTrue(powerAmpArm); //should not be negated for driver control
    operatorController.start().whileTrue(powerClimber); //should be negated for driver control


    operatorController.pov(0).onTrue(soloClimb);
    operatorController.pov(270).onTrue(goToZero);
    operatorController.pov(180).onTrue(fullRetract);
    operatorController.back().onTrue(new InstantCommand(() -> climber.zeroBoth(), climber));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void configureCharacterizationBindings() {
    // The methods below return Command objects
    driverController.rightTrigger().whileTrue(drivetrain.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
    driverController.leftTrigger().whileTrue(drivetrain.sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
    driverController.x().whileTrue(drivetrain.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
    driverController.y().whileTrue(drivetrain.sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }

  public void onDisable() {

    climber.stop();
    flywheel.stop();
    intake.stop();
    linkage.disableBrakeMode();
    ampArm.disableBrakeMode();

    linkage.stop();
    drivetrain.robotCentricDrive(0, 0, 0);
    if (!Objects.isNull(ampArm)) {
      ampArm.stopBoth();
    }
    if (!Objects.isNull(ampIntake)) {
      ampIntake.stop();
    }
  }

  public void onTeleInit() {
    drivetrain.configNeutralMode(NeutralModeValue.Brake);
    linkage.enableBrakeMode();
    ampArm.enableBrakeMode();
  }

  private double fetchAllianceNum() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return -45.0;
    } else {
      return 45.0;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    // An example command will be run in autonomous
  }
}
