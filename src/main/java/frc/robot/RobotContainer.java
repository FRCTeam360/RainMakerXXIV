// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotType;
import frc.robot.commands.DiagonalSensorIntake;
import frc.robot.commands.DriveFieldCentricFacingAngle;
import frc.robot.commands.RunExtendIntake;
import frc.robot.commands.RydarsSpinup;
import frc.robot.commands.SetClimbers;
import frc.robot.commands.ScoreInAmp;
import frc.robot.commands.SetArmWrist;
import frc.robot.commands.PowerIntakeReversed;
import frc.robot.commands.PowerIntake;
import frc.robot.commands.PowerLinkage;
import frc.robot.commands.PristineIntakeCommand;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SetLinkage;
import frc.robot.commands.ShootInSpeaker;
import frc.robot.commands.ShootingPrepRyRy;
import frc.robot.commands.SnapDrivebaseToAngle;
import frc.robot.commands.TrapClimb;
import frc.robot.commands.TrapSetUp;
import frc.robot.commands.TrapSetUpTheSequel;
import frc.robot.commands.StopClimber;
import frc.robot.commands.TuneFlywheel;
import frc.robot.commands.TuneSwerveDrive;
import frc.robot.commands.PowerFlywheel;
import frc.robot.commands.RobotOrientedDrive;
import frc.robot.commands.AmpArmGoToZero;
import frc.robot.commands.AmpArmNote;
import frc.robot.commands.AmpArmStop;
import frc.robot.commands.AutoIntakeCOmmand;
import frc.robot.commands.AutoPowerCenterNote;
import frc.robot.commands.BasicClimb;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.HoldArmPosition;
import frc.robot.commands.HomeAmpArmWrist;
import frc.robot.commands.IntakeCOmmand;
import frc.robot.commands.LevelClimbers;
import frc.robot.commands.ClimberPIDTuner;
import frc.robot.commands.LinkageSetpoint;
import frc.robot.commands.LinkageToAmpHandoff;
import frc.robot.commands.PointDrivebaseAtTarget;
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
import frc.robot.hardware.VisionIOLimelight;
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
import frc.robot.subsystems.Vision;
import frc.robot.utils.CommandFactory;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.Objects;
import java.util.Optional;

import javax.management.InstanceNotFoundException;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
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
  private final CommandXboxController testController = new CommandXboxController(Constants.TEST_CONTROLLER);

  final double MAX_SPEED_MPS = Constants.MAX_SPEED_MPS; // used to be 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI * 3; // Half a rotation per second max angular velocity
  // dio ports
  private DigitalInput zeroButton = new DigitalInput(Constants.LINKAGE_ZERO_BUTTON_PORT);
  private DigitalInput brakeButton = new DigitalInput(Constants.LINKAGE_BRAKE_TOGGLE_BUTTON_PORT);

  // subsystems
  private CommandSwerveDrivetrain drivetrain; // My drivetrain
  private Flywheel flywheel;
  private Linkage linkage;
  private PristineIntakeCommand intakeMe;
  private Intake intake;
  private Climber climber;
  private AmpArm ampArm;
  private AmpIntake ampIntake;
  private Vision vision;
  private PointDrivebaseAtTarget pointDrivebaseAtTarget;

  private CommandFactory commandFactory;

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private ShootInSpeaker shootRoutine;
  private RunExtendIntake runExtendIntake;
  private DiagonalSensorIntake diagonalSensorIntakeCloseShot;
  private Command shootAtSpeakerVision;
  private Command spinUpSpeakerVision;

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
  private DriveFieldCentricFacingAngle passFromSourceAngle;
  private SnapDrivebaseToAngle snapDrivebaseToAngle;
  private ClimberPIDTuner pidTuner;
  private SetClimbers maxExtend;
  private SetClimbers minExtend;

  private AutoIntakeCOmmand longerinny;
  // private SetLinkageTalon setLinkageTalon = new SetLinkageTalon(linkage);
  private SetLinkage setLinkage;
  private IntakeCOmmand ryryinny;
  private SetLinkage stowLinkage;
  private LinkageSetpoint linkageSetpoint;
  private TuneFlywheel tuneFlywheel;
  private ShootInSpeaker shootFromSubwoofer;
  private ShootInSpeaker shootFromPodium;
  private Command shootFromSubwooferSpinUp;
  private TuneSwerveDrive tuneSwerveDrive;
  private AutoPowerCenterNote autoPowerCenterNote;
  private PowerAmpIntakeReverse powerAmpIntakeReverse;
  private AmpArmNote ampArmNote;
  private IntakeCOmmand inny;
  private AutoIntakeCOmmand autoinny;
  private ScoreInAmp scoreInAmp;
  private LinkageToAmpHandoff linkageToAmpHandoff;
  private AmpArmStop ampArmStop;
  private BasicClimb basicClimb;
  private TrapSetUpTheSequel sequal;
  private Command passUnderStage;
  private RydarsSpinup rydarSubwoof;

  private SetClimbers goToZero;
  private SetClimbers fullRetract;
  private SetClimbers soloRaise;
  private ShootingPrepRyRy kiki;
  private SetClimbers soloRetract;

  private SetLinkage deploy;
  private FieldOrientedDrive fieldOrientedSlowGuy;

  private StopClimber stopClimber;

  private HomeAmpArmWrist homeAmpArmWrist;
  private AmpArmGoToZero ampArmGoToZero;
  private Command visionBoy;

  private TrapSetUp trapDrive;
  private TrapClimb trapClimb;
  private ShootingPrepRyRy subwoofShotRy;

  private HoldArmPosition holdArmPosition;
  private SetArmWrist ampSetpoint;
  private SetArmWrist homeArmWrist;
  private SetArmWrist stowArm;

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
        ampArm = new AmpArm(new AmpArmIOTalonFX(zeroButton, brakeButton));
        ampIntake = new AmpIntake(new AmpIntakeIOSparkMax());

        drivetrain = PracticebotConstants.DriveTrain; // My drivetrain
        drivetrain.configNeutralMode(NeutralModeValue.Brake);
        vision = new Vision(new VisionIOLimelight(
            Constants.VisionConstants.PRACTICE_LIMELIGHT_NAME,
            Constants.VisionConstants.PRACTICE_LIMELIGHT_HEIGHT_METERS,
            Constants.VisionConstants.PRACTICE_LIMELIGHT_PITCH_DEGREES,
            Constants.VisionConstants.PRACTICE_LIMELIGHT_HEIGHT_FUDGE_FACTOR_METERS,
            Constants.VisionConstants.PRACTICE_LIMELIGHT_PITCH_FUDGE_FACTOR_DEGREES));
        break;
      case COMPETITION:
        drivetrain = CompBotConstants.DriveTrain;
        drivetrain.configNeutralMode(NeutralModeValue.Brake);

        flywheel = new Flywheel(new FlywheelIOSparkFlex());
        intake = new Intake(new IntakeIOSparkFlex());
        ampArm = new AmpArm(new AmpArmIOTalonFX(zeroButton, brakeButton));
        ampIntake = new AmpIntake(new AmpIntakeIOSparkMax());
        climber = new Climber(new ClimberIOSparkMax());
        linkage = new Linkage(new LinkageIOTalonFX(zeroButton, brakeButton));
        vision = new Vision(new VisionIOLimelight(
            Constants.VisionConstants.COMP_LIMELIGHT_NAME,
            Constants.VisionConstants.COMP_LIMELIGHT_HEIGHT_METERS,
            Constants.VisionConstants.COMP_LIMELIGHT_PITCH_DEGREES,
            Constants.VisionConstants.COMP_LIMELIGHT_HEIGHT_FUDGE_FACTOR_METERS,
            Constants.VisionConstants.COMP_LIMELIGHT_PITCH_FUDGE_FACTOR_DEGREES));
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

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    commandFactory = new CommandFactory(climber, drivetrain, intake, flywheel, linkage, ampArm, vision);
    fieldOrientedDrive = new FieldOrientedDrive(drivetrain, linkage, ampArm, false);
    fieldOrientedSlowGuy = new FieldOrientedDrive(drivetrain, linkage, ampArm, true);

    passFromSourceAngle = new DriveFieldCentricFacingAngle(drivetrain, 225.0, 315.0);
    robotOrientedDrive = new RobotOrientedDrive(drivetrain);
    runExtendIntake = commandFactory.runExtendIntake();
    autoPowerCenterNote = new AutoPowerCenterNote(ampArm, intake, linkage, flywheel, 177.0);
    powerCenterNoteIntakeRoutine = commandFactory.powerCenterNote();
    subwoofShotRy = new ShootingPrepRyRy(linkage, flywheel, ampArm, 177.0, 5000.0);
    sequal = new TrapSetUpTheSequel(linkage, ampArm, drivetrain, climber);
    intakeMe = new PristineIntakeCommand(intake, linkage, ampArm, 145.0);
    visionBoy = commandFactory.shootAtSpeakerVision();
    kiki = new ShootingPrepRyRy(linkage, flywheel, ampArm, 153.0, 7000.0);

    powerIntakeReversed = new PowerIntakeReversed(intake);
    powerIntake = new PowerIntake(intake);
    powerFlywheel = new PowerFlywheel(flywheel);
    powerClimber = new PowerClimber(climber);
    shootRoutine = new ShootInSpeaker(ampArm, linkage, flywheel, drivetrain, intake, 174.0, 5000.0, 90.0);
    maxExtend = new SetClimbers(climber, 70.0);
    passUnderStage = commandFactory.spinUpForUnderPassAndShoot();
    minExtend = new SetClimbers(climber, -35.0);
    // levelClimbers = new LevelClimbers(climber, drivetrain);
    tuneFlywheel = new TuneFlywheel(flywheel);
    linkageSetpoint = new LinkageSetpoint(linkage, ampArm);
    powerLinkage = new PowerLinkage(linkage, ampArm);
    stowLinkage = commandFactory.stowLinkage();
    powerAmpIntakeReverse = new PowerAmpIntakeReverse(ampIntake);
    inny = new IntakeCOmmand(intake, linkage, ampArm, vision, 110.0, true);
    autoinny = new AutoIntakeCOmmand(intake, linkage, ampArm, vision, 177.0, true);
    longerinny = new AutoIntakeCOmmand(intake, linkage, ampArm, vision, 144.0, true);
    ryryinny = new IntakeCOmmand(intake, linkage, ampArm, vision, 0.0, false);
    powerLinkage = commandFactory.powerLinkage();
    shootRoutine = commandFactory.shootInSpeaker(177.0, 6000.0);
    shootFromSubwooferSpinUp = commandFactory.shootFromSubwooferSpinUp();
    // autoCenterNote = commandFactory.shootInSpeaker(160.0, 6000.0);
    shootFromSubwoofer = commandFactory.shootFromSubwoofer();
    rydarSubwoof = new RydarsSpinup(linkage, ampArm, flywheel, 177.0, 5000.0);
    shootFromPodium = commandFactory.shootFromPodium();
    pointDrivebaseAtTarget = commandFactory.pointDriveBaseAtTarget();
    shootAtSpeakerVision = commandFactory.shootAtSpeakerVision();
    spinUpSpeakerVision = commandFactory.spinUpSpeakerVision();

    deploy = commandFactory.deploy();

    trapDrive = new TrapSetUp(drivetrain, linkage, ampArm, climber);
    trapClimb = new TrapClimb(ampArm, climber, linkage);

    goToZero = commandFactory.setClimberShouldFinish(0);
    soloRaise = commandFactory.setClimberShouldntFinish(40);
    soloRetract = commandFactory.setClimberShouldntFinish(-20);

    fullRetract = commandFactory.setClimberShouldntFinish(-58);

    stopClimber = new StopClimber(climber);

    pointDrivebaseAtTarget = new PointDrivebaseAtTarget(drivetrain, vision);
    snapDrivebaseToAngle = new SnapDrivebaseToAngle(drivetrain);

    // COMMENT OUT tuneSwerveDrive WHEN NOT USING, IT WILL SET YOUR SWERVE DRIVE
    // CONSTANTS TO 0 WHEN CONSTRUCTED
    // tuneSwerveDrive = new TuneSwerveDrive(drivetrain);
    if (!Objects.isNull(ampArm)) {
      powerAmpArm = new PowerAmpArm(ampArm, linkage);
      ampArmStop = commandFactory.ampArmStop();
      ampArmGoToZero = new AmpArmGoToZero(ampArm, linkage);
      holdArmPosition = new HoldArmPosition(ampArm, linkage);
      stowArm = new SetArmWrist(ampArm, linkage, -75, 65);
      ampSetpoint = new SetArmWrist(ampArm, linkage, 95.0, 135.0);
      homeArmWrist = new SetArmWrist(ampArm, linkage, -6.0, 80.0);
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
    NamedCommands.registerCommand("Intake", autoinny);

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
    NamedCommands.registerCommand("kiki linkage long prep",
        new InstantCommand(() -> linkage.setAngle(149, ampArm)));
    NamedCommands.registerCommand("stay out of way shot",
        new ShootInSpeaker(ampArm, linkage, flywheel, intake, 150, 7000.0));
    NamedCommands.registerCommand("kikiSimpleShoot",
        new ShootInSpeaker(ampArm, linkage, flywheel, intake, 149, 7000.0));
    NamedCommands.registerCommand("long shot inny", longerinny);
    NamedCommands.registerCommand("last guy", new ShootInSpeaker(ampArm, linkage, flywheel, intake, 153, 7000.0));
    NamedCommands.registerCommand("blue linkage long prep",
        new InstantCommand(() -> linkage.setAngle(148, ampArm), linkage));
    NamedCommands.registerCommand("blue stay out of way shot",
        new ShootInSpeaker(ampArm, linkage, flywheel, intake, 148, 7000.0));
    NamedCommands.registerCommand("blue last guy",
        new ShootInSpeaker(ampArm, linkage, flywheel, intake, 151.5, 7000.0));
    NamedCommands.registerCommand("kiki shot", kiki);
    NamedCommands.registerCommand("Turn", pointDrivebaseAtTarget);

    NamedCommands.registerCommand("vision shoot", visionBoy);
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
    if (Objects.nonNull(ampArm)) {
      ampArm.setDefaultCommand(holdArmPosition);
    }
    climber.setDefaultCommand(powerClimber);
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
   * 
   */

  private void configureBindings() {
    // DRIVER CONTROLS DO NOT DELETE JUST COMMENT OUT
    driverController.leftBumper().whileTrue(powerIntakeReversed);
    driverController.rightBumper().whileTrue(inny);
    driverController.b().onTrue(stowLinkage);
    driverController.a().and(driverController.rightTrigger().negate()).whileTrue(shootFromSubwooferSpinUp);
    driverController.x().whileTrue(snapDrivebaseToAngle);

    driverController.rightTrigger().and(driverController.leftTrigger().negate()).and(driverController.back().negate())
        .whileTrue(shootFromSubwoofer);
    driverController.rightTrigger().and(driverController.leftTrigger()).and(driverController.back().negate())
        .whileTrue(shootAtSpeakerVision);
    driverController.leftTrigger().and(driverController.rightTrigger().negate()).and(driverController.back().negate())
        .whileTrue(spinUpSpeakerVision);

    driverController.start().whileTrue(commandFactory.spinUpForOverPassAndShoot());

    driverController.back().and(driverController.rightTrigger().negate()).and(driverController.leftTrigger().negate())
        .whileTrue(commandFactory.spinUpForUnderPass());
    driverController.back().and(driverController.rightTrigger()).and(driverController.leftTrigger().negate())
        .whileTrue(commandFactory.spinUpForUnderPassAndShoot());
    driverController.pov(180).whileTrue(new InstantCommand(() -> drivetrain.zero(), drivetrain));
    driverController.pov(0).toggleOnTrue(deploy);
    driverController.pov(90).whileTrue(powerIntake);

    // OPERATOR CONTROLS DO NOT DELETE JUST COMMENT OUT

    operatorController.leftBumper().whileTrue(powerAmpIntakeReverse);
    operatorController.rightBumper().whileTrue(powerAmpIntake);

    if (Objects.nonNull(ampArm)) {
      driverController.y().whileTrue(trapDrive.andThen(sequal.andThen(robotOrientedDrive)));

      operatorController.x().onTrue(linkageToAmpHandoff.alongWith(fieldOrientedSlowGuy));
      operatorController.a().toggleOnTrue(ampSetpoint);

      operatorController.y().toggleOnTrue(homeArmWrist);
      operatorController.b().toggleOnTrue(stowArm);

      // operatorController.a().toggleOnTrue(new InstantCommand(() ->
      // ampIntake.runIntake(.5)));
      operatorController.pov(90).toggleOnTrue(trapClimb);

      // operatorController.pov(90).onTrue(homeAmpArmWrist);
      // operatorController.pov(180).onTrue(ampArmGoToZero);
    }

    // operatorController.b().toggleOnTrue(trapClimb);
    operatorController.start().whileTrue(stopClimber);
    operatorController.start().whileTrue(powerAmpArm);
    operatorController.pov(0).onTrue(soloRaise);
    operatorController.pov(270).onTrue(goToZero);
    // operatorController.leftStick().onTrue(fullRetract);
    operatorController.pov(180).toggleOnTrue(soloRetract);
    operatorController.back().onTrue(new InstantCommand(() -> climber.zeroBoth(), climber));
    operatorController.leftStick().onTrue(new InstantCommand(() -> {
      ampArm.setArm78();
      ampArm.setWrist70();
    }, ampArm));

    drivetrain.registerTelemetry(logger::telemeterize);
    configureTestBindings();
  }

  public void configureTestBindings() {
    testController.rightBumper().whileTrue(inny);
    testController.leftTrigger().whileTrue(commandFactory.tuneLinkageSetpoint());
    testController.rightTrigger().whileTrue(intake.runEnd(() -> intake.run(1.0), () -> intake.run(0.0)));
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
  }

  private double fetchAllianceNum() {
    return -45.0;
    // if (DriverStation.getAlliance().get() == Alliance.Blue) {
    // return -45.0;
    // } else {
    // return 45.0;
    // }
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
