// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotType;
import frc.robot.commands.RunExtendIntake;
import frc.robot.commands.PowerIntakeReversed;
import frc.robot.commands.PowerIntake;
import frc.robot.commands.PowerLinkage;
import frc.robot.commands.SetIntake;
import frc.robot.commands.SetLinkage;
import frc.robot.commands.ShootInSpeaker;
import frc.robot.commands.TuneFlywheel;
import frc.robot.commands.TuneSwerveDrive;
import frc.robot.commands.PowerFlywheel;
import frc.robot.commands.RobotOrientedDrive;
import frc.robot.commands.AutoPowerCenterNote;
import frc.robot.commands.FieldOrientedDrive;
import frc.robot.commands.LevelClimbers;
import frc.robot.commands.LinkageSetpoint;
import frc.robot.commands.PowerAmpArm;
import frc.robot.commands.PowerAmpIntake;
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

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Translation2d;
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
  // subsystems
  private CommandSwerveDrivetrain drivetrain; // My drivetrain
  private Flywheel flywheel;
  private Linkage linkage;
  private Intake intake;
  private Climber climber;
  private AmpArm ampArm;
  private AmpIntake ampIntake;
  // private final Climber climber = new Climber(new ClimberIOSparkMax());
  private CommandFactory commandFactory;

  // subsystems

  public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  private ShootInSpeaker shootRoutine;
  // tele commands
  private RunExtendIntake runExtendIntake;
  private PowerCenterNote powerCenterNoteIntakeRoutine;
  private PowerIntakeReversed powerIntakeReversed;
  private PowerIntake powerIntake;
  private PowerFlywheel powerFlywheel;
  private PowerClimber powerClimber;
  private LevelClimbers levelClimbers;
  private PowerAmpArm powerAmpArm;
  private PowerAmpIntake powerAmpIntake;
  // private PowerLinkage powerLinkage = new PowerLinkage(linkage);
  private ShuffleboardTab diagnosticTab;
  private FieldOrientedDrive fieldOrientedDrive; 
  private RobotOrientedDrive robotOrientedDrive; 
  private PowerLinkage powerLinkage;
  private SetLinkage setLinkage;
  private SetLinkage stowLinkage;
  private LinkageSetpoint linkageSetpoint;
  private TuneFlywheel tuneFlywheel;
  private ShootInSpeaker shootFromSubwoofer;
  private ShootInSpeaker shootFromFar;
  private TuneSwerveDrive tuneSwerveDrive;
  private AutoPowerCenterNote autoPowerCenterNote;

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
        intake = new Intake(new IntakeIOSparkMax());
        linkage = new Linkage(new LinkageIOTalonFX());
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
        ampArm = new AmpArm(new AmpArmIOTalonFX());
        ampIntake = new AmpIntake(new AmpIntakeIOSparkMax());
        climber = new Climber(new ClimberIOSparkMax());
        linkage = new Linkage(new LinkageIOTalonFX());
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
    //configureCharacterizationBindings();
    configureDefaultCommands();
  }

  private final void initializeCommands() {
    commandFactory = new CommandFactory(climber, drivetrain, intake, flywheel, linkage);
    fieldOrientedDrive = new FieldOrientedDrive(drivetrain);
    robotOrientedDrive = new RobotOrientedDrive(drivetrain);
    runExtendIntake = commandFactory.runExtendIntake();
    //autoPowerCenterNote = new AutoPowerCenterNote(intake, linkage);
    powerCenterNoteIntakeRoutine = commandFactory.powerCenterNote();
    powerIntakeReversed = new PowerIntakeReversed(intake);
    powerIntake = new PowerIntake(intake);
    powerFlywheel = new PowerFlywheel(flywheel);
    powerClimber = new PowerClimber(climber);
    levelClimbers = new LevelClimbers(climber, drivetrain);
    tuneFlywheel = new TuneFlywheel(flywheel);
    linkageSetpoint = new LinkageSetpoint(linkage);
    stowLinkage = commandFactory.stowLinkage();
    shootRoutine = commandFactory.shootInSpeaker(174.0, 6000.0);
    //autoCenterNote = commandFactory.shootInSpeaker(160.0, 6000.0);
    shootFromSubwoofer = commandFactory.shootFromSubwoofer();
    shootFromFar = commandFactory.shootFromFar();
    // COMMENT OUT tuneSwerveDrive WHEN NOT USING, IT WILL SET YOUR SWERVE DRIVE CONSTANTS TO 0 WHEN CONSTRUCTED
    //tuneSwerveDrive = new TuneSwerveDrive(drivetrain);
    if(!Objects.isNull(ampArm)){
      powerAmpArm = new PowerAmpArm(ampArm);
    }
    if(!Objects.isNull(ampIntake)){
      powerAmpIntake = new PowerAmpIntake(ampIntake);
    }
    // powerAmpArm = new PowerAmpArm(ampArm);
    // powerAmpIntake = new PowerAmpIntake(ampIntake);
    
    Command shootRoutineWithDrivetrain = new ShootInSpeaker(linkage, flywheel, drivetrain, intake, 0.0, 5000.0, 0.0);
    NamedCommands.registerCommand("Intake", runExtendIntake);
    NamedCommands.registerCommand("Auto Center Note", new AutoPowerCenterNote(intake, linkage, flywheel, 163));
    NamedCommands.registerCommand("Wait1", new WaitCommand(1));
    NamedCommands.registerCommand("Shoot", shootRoutineWithDrivetrain);
    NamedCommands.registerCommand("Rotate", drivetrain.turntoCMD(false, 45.0, 0, 0));
    NamedCommands.registerCommand("Shoot without drivetrain", shootRoutine);
    NamedCommands.registerCommand("Shoot from subwoofer", shootFromSubwoofer);
    NamedCommands.registerCommand("Spinny", new PowerFlywheel(flywheel));    
    NamedCommands.registerCommand("AutoShot1", new ShootInSpeaker(linkage, flywheel, intake, 163.0, 6500.0));
    NamedCommands.registerCommand("extend linkage", new InstantCommand(() -> linkage.setAngle(0), linkage));

    // NamedCommands.registerCommand("Intake", runExtendIntake);
    // NamedCommands.registerCommand("Wait1", new WaitCommand(1));
    // NamedCommands.registerCommand("Wait", new WaitCommand(2));
    // NamedCommands.registerCommand("Shoot", shootRoutine);
    // NamedCommands.registerCommand("Rotate", drivetrain.turntoCMD(false, 45.0, 0, 0));
    // NamedCommands.registerCommand("Shoot without drivetrain", new ShootInSpeaker(linkage, flywheel, drivetrain, intake, MAX_SPEED_MPS, MaxAngularRate, MAX_SPEED_MPS));
    powerLinkage = new PowerLinkage(linkage);
    // fieldOrientedDrive = new FieldOrientedDrive();
    // robotOrientedDrive = new RobotOrientedDrive();
    // drivetrain = TunerConstants.DriveTrain; // My drivetrain
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(fieldOrientedDrive);
    // linkage.setDefaultCommand(powerLinka$ge);
    // // climber.setDefaultCommand();
    //  drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(
    //         () -> drive.withVelocityX(MathUtil.applyDeadband(-driverController.getLeftY(), 0.1) * MAX_SPEED_MPS) //drive forward with negative y
    //             // negative Y (forward)
    //             .withVelocityY(MathUtil.applyDeadband(-driverController.getLeftX(), 0.1) * MAX_SPEED_MPS) // drive left with negative x
    //             .withRotationalRate(MathUtil.applyDeadband(-driverController.getRightX(), 0.1) * MaxAngularRate) // drive counterclockwise with negative x                                                                                                  
    //flywheel.setDefaultCommand(setFlywheel);
    //intake.setDefaultCommand(runIntake);
    //linkage.setDefaultCommand(powerLinkage);
  //  drivetrain.setDefaultCommand(fieldOrientedDrive);
    // flywheel.setDefaultCommand(powerFlywheel);
    // drivetrain.setDefaultCommand(fieldOrientedDrive);
    //climber.setDefaultCommand(powerClimber);
   // linkage.setDefaultCommand(powerLinkage);
    // ampArm.setDefaultCommand(powerAmpArm);
    linkage.setDefaultCommand(linkageSetpoint);
    flywheel.setDefaultCommand(tuneFlywheel);
    //linkage.setDefaultCommand(stowLinkage);
    
    // if(!Objects.isNull(ampArm)){
    //   ampArm.setDefaultCommand(powerAmpArm);
    // }
    // if(!Objects.isNull(ampIntake)){
    //   ampIntake.setDefaultCommand(powerAmpIntake);
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
    driverController.a().whileTrue(shootFromSubwoofer);
    driverController.x().whileTrue(shootFromFar);
    driverController.y().onTrue(stowLinkage);
    driverController.b().whileTrue(new InstantCommand(() -> drivetrain.zero(), drivetrain));
    driverController.rightBumper().toggleOnTrue(powerCenterNoteIntakeRoutine);
    operatorController.leftTrigger(.15).whileTrue(powerIntakeReversed);
    operatorController.rightTrigger(.15).whileTrue(powerIntake);
    // operatorController.y().onTrue(new SetLinkageTalon(linkage, 0.0));
    // operatorController.b().onTrue(new SetLinkageTalon(linkage, 90.0));
    // operatorController.a().whileTrue(powerFlywheel);
    operatorController.a().toggleOnTrue(powerCenterNoteIntakeRoutine);
    operatorController.b().whileTrue(powerLinkage);
    operatorController.start().and(() -> operatorController.back().getAsBoolean()).whileTrue(new InstantCommand(() -> linkage.setEncoderTo174()));
    // operatorController.x().whileTrue(shootRoutine);
    // operatorController.a().toggleOnTrue(runExtendIntake);

    // operatorController.y().toggleOnTrue(powerAmpIntake);

   // operatorController.a().onTrue(levelClimbers);
  //  operatorController.b().onTrue(new InstantCommand(() -> climber.zeroBoth(), climber));

    // driverController.x().whileTrue(new InstantCommand(() ->
    // drivetrain.zero(),drivetrain));

    // drivetrain.registerTelemetry(logger::telemeterize);
  }
  public void configureCharacterizationBindings(){
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
